# Anti-Cogging and Motor Diagnostic Analysis (Continuous DFT-128 Method)

This document describes the harmonic-based anti-cogging compensation and motor diagnostic system for the OpenFFBoard (TMC4671). The system uses a **Continuous Discrete Fourier Transform (DFT)** integration with multi-RPM gain scheduling, speed-dependent scale curves, velocity-based phase advance, and harmonic waveshaping.

---

## 1. Process Overview

### 1.1 Encoder Identification & System Identification (SysId)

Before DFT acquisition, the system performs a deterministic identification of the motor's physical parameters to calculate ideal PID gains (using CMSIS-DSP):

- **Encoder Profiling**: Audits `enc_cpr` to mathematically decimate the PID execution rate.
- **Breakout Torque**: Measures minimum torque to overcome static friction.
- **Mechanical Inertia ($J$)**: $J = \tau / \alpha$ from a constant torque pulse (150 ms).
- **Viscous Friction ($B$)**: $B = \tau / \omega$ from steady-state 30 RPM velocity hold (2000 ms).
- **IMC Pole Placement**: Critically damped PID ($\zeta = 1.0$) with bandwidth $f_{bw}$ dynamically degraded by inertia.

### 1.2 Multi-RPM DFT Acquisition

The system runs **three iterative DFT passes** at three different sweep speeds, producing three independent cogging maps. The DFT loop is factored into a reusable lambda (`runDftPass`) with local reference aliases so the unchanged loop body binds to whichever target array the caller passes.

| Pass | RPM | Purpose |
| :--- | :--- | :--- |
| **RPM#1** (Low) | 3 RPM (or encoder-dependent) | Baseline cogging at near-static speed — minimal current-loop attenuation or phase lag |
| **RPM#2** (Hi) | `COGGING_CALIB_HI_RPM` (default 30) | Absorbs closed-loop torque-path distortion (current-loop attenuation + phase lag) in the mid-speed band |
| **RPM#3** (Ultra) | `COGGING_CALIB_ULTRA_RPM` (default 100) | Absorbs high-speed dynamics for the upper RPM range |

Each pass uses identical DFT math: 128 harmonics, 3 iterations with CW+CCW sweeps and Piccoli phase averaging, top-20 insertion sort, and phasor-addition residual reduction on iterations 2-3.

#### DFT Sweep Details

- **Per-direction integration**: Exactly 360° of displacement after a 1500 ms warmup period.
- **Decimated DFT accumulation**: Every 4th loop cycle (~1 kHz) to prevent float accumulator overflow at 4 kHz.
- **Recursive complex multiplication**: $e^{i(k+1)\theta} = e^{ik\theta} \cdot e^{i\theta}$ — only one `arm_sin_cos_f32` call per sample for all 128 harmonics.
- **Real-time inertia feedforward**: $iq_{\text{inertia}} = J \cdot \alpha$ subtracted from measured torque before DFT.
- **Cogging feed-forward during sweeps**: The running harmonic table is applied as FF, so later iterations measure only the residual (phasor-added onto the table).

#### Encoder Profiling Constants

| Encoder Class | CPR Range | PID Rate | Kp Penalty | Calib RPM |
| :--- | :--- | :--- | :--- | :--- |
| **High-Res** | > 50,000 | 4 kHz | 1.0× | 3.0 |
| **Medium-Res** | 20,000–50,000 | 1 kHz | 0.5× | 6.0 |
| **Low-Res** | < 20,000 | 500 Hz | 0.2× | 12.0 |

### 1.3 Scale Calibration (Optional)

If `COGGING_SCALE_SWEEP` is defined, a gradient-descent sweep across 14 RPM breakpoints (3–100 RPM) optimizes `cogging_scale` at each speed, populating the 24-point scale curve.

---

## 2. Runtime Compensation

### 2.1 Multi-RPM Waveform Blending

Implemented in `TMC4671::turn()`. The three maps are reconstructed independently and blended **in the torque domain** (not the harmonic/phasor domain), so the maps do not need to share the same harmonic orders:

$$ \alpha_{12} = \text{clamp}\!\left(\frac{\text{rpm} - \text{rpm}_{\text{low}}}{\text{rpm}_{\text{hi}} - \text{rpm}_{\text{low}}},\,0,\,1\right) $$

$$ \alpha_{23} = \text{clamp}\!\left(\frac{\text{rpm} - \text{rpm}_{\text{hi}}}{\text{rpm}_{\text{ultra}} - \text{rpm}_{\text{hi}}},\,0,\,1\right) $$

| RPM range | Compensation |
| :--- | :--- |
| $\text{rpm} \le \text{rpm}_{\text{low}}$ | 100% RPM#1 |
| $\text{rpm}_{\text{low}} < \text{rpm} \le \text{rpm}_{\text{hi}}$ | $(1-\alpha_{12})\,\text{RPM}\#1 + \alpha_{12}\,\text{RPM}\#2$ |
| $\text{rpm}_{\text{hi}} < \text{rpm} \le \text{rpm}_{\text{ultra}}$ | $(1-\alpha_{23})\,\text{RPM}\#2 + \alpha_{23}\,\text{RPM}\#3$ |
| $\text{rpm} > \text{rpm}_{\text{ultra}}$ | 100% RPM#3 |

The crossover points (`blend_low_rpm`, `blend_high_rpm`, `blend_ultra_rpm`) default to the calibration sweep speeds but can be retuned live via the configurator or serial commands.

### 2.2 Velocity-Based Phase Advance

A position offset proportional to RPM compensates for high-speed loop latency. Stored as a 24-point curve (degrees vs RPM) in EEPROM. Applied before waveform lookup:

$$\theta' = \theta + \text{sign}(\omega) \cdot \frac{\text{adv}(\text{rpm})}{360}$$

### 2.3 Speed-Dependent Scale Curve

A 24-point curve interpolates `cogging_scale` by measured RPM to counteract current-loop amplitude rolloff at higher speeds. Stored in EEPROM alongside the phase-advance curve.

### 2.4 Harmonic Waveshaping

A shaping term applied to the dominant harmonic to thin peaks / steepen slopes:

$$T_{\text{shaped}} = T_{\text{comp}} - s \cdot A_{\text{dom}} \cdot \sin\!\big(m \cdot (k_{\text{dom}} \cdot \theta + \phi_{\text{dom}}) + \phi_{\text{trim}}\big)$$

- $s$: shaping factor (−1.0 to +1.0), positive thins peaks
- $m$: harmonic multiplier (default 3)
- $\phi_{\text{trim}}$: extra phase offset

### 2.5 Full Runtime Formula

$$T_{\text{comp}}(\theta, \omega) = \text{scale}(\omega) \cdot \left[ \text{blend}\!\big(T_1(\theta'),\, T_2(\theta'),\, T_3(\theta'),\, \omega\big) - \text{shape}(T_{\text{dom}}) \right]$$

$$T_{\text{final}} = T_{\text{requested}} + T_{\text{comp}}$$

---

## 3. Storage Architecture

### 3.1 Flash Cogging Region

Three maps per TMC driver, stored in a dedicated flash sector (sector 4 at `0x08010000`):

| Slot | Contents |
| :--- | :--- |
| 0–2 | Low-RPM maps (drv0, drv1, drv2) |
| 3–5 | Hi-RPM maps |
| 6–8 | Ultra-RPM maps |

Each slot: `COGGING_TABLE_SIZE = COGGING_HARMONICS_COUNT × 12 = 240` bytes (20 harmonics × {float amp, float phase, uint16 order}).

Static buffer in `flash_helpers.cpp`: `cogging_flash_buffer[4096]` (9 × 240 = 2160 bytes, rounded up).

Read-modify-write via `Flash_WriteCoggingTable()` preserves other tables during writes.

### 3.2 EEPROM Settings

Per-driver settings persisted in the EEPROM emulation region:

| Address Range | Content |
| :--- | :--- |
| `0x32D–0x32F` | Cogging enable, scale, dynamic offset |
| `0x420–0x4AF` | 24-pt scale curve + 24-pt phase advance curve (all 3 drivers) |
| `0x4B0–0x4B8` | H3 waveshaping per driver |
| `0x4B9–0x4BE` | Blend crossover + hi validity per driver |
| `0x4BF–0x4C4` | Blend crossover + ultra validity per driver |

`NB_OF_VAR = 399`.

### 3.3 RAM Footprint

| Component | Size | Lifecycle |
| :--- | :--- | :--- |
| `cogging_harmonics[]` (low) | 240 B | Permanent |
| `cogging_harmonics_hi[]` | 240 B | Permanent |
| `cogging_harmonics_ultra[]` | 240 B | Permanent |
| `cw_store[]` / `ccw_store[]` | 480 B | Permanent |
| Scale curve values | 96 B | Permanent |
| Phase advance curve values | 96 B | Permanent |
| DFT accumulators (128 harmonics) | ~2 KB | Calibration only (heap) |
| Full `CoggingCalibData` | ~17 KB | Calibration only (heap) |
| **Total permanent** | ~1.4 KB | — |

---

## 4. Commands Reference

### Multi-RPM Commands

| Command | Access | Format | Description |
| :--- | :--- | :--- | :--- |
| `coggingHarmonics` | GET | `order:amp:phase,…` | Low-RPM harmonic table (phase ×1000) |
| `coggingHarmonicsHi` | GET | `order:amp:phase,…` | Hi-RPM (RPM#2) harmonic table |
| `coggingHarmonicsUltra` | GET | `order:amp:phase,…` | Ultra-RPM (RPM#3) harmonic table |
| `coggingBlendHigh` | GET/SET | RPM ×100 | RPM#2 crossover (set 0 or 0xFFFF to ignore) |
| `coggingBlendUltra` | GET/SET | RPM ×100 | RPM#3 crossover |
| `coggingHiValid` | GET | `0` or `1` | RPM#2 map validity |
| `coggingUltraValid` | GET | `0` or `1` | RPM#3 map validity |
| `coggingClearHi` | GET | — | Clears RPM#2+3 maps & marks invalid |

### Tuning Commands

| Command | Access | Description |
| :--- | :--- | :--- |
| `scaleCurve` | GET/SETADR | 24-pt scale curve (value ×1000) |
| `phaseAdvCurve` | GET/SETADR | 24-pt phase advance in degrees (value ×100) |
| `coggingH3` | GET/SETADR | Waveshaping: `shaping:phaseTrim:mult` |
| `coggingScale` | GET/SET | Static scale (×10000) |
| `cogging` | GET/SET | Cogging enable/disable |
| `calibrateCogging` | GET | Start multi-pass calibration |

---

## 5. Configuration Constants

```c
// TMC4671.h
#define COGGING_CALIB_TIME_PER_REV_S    20      // Low-RPM sweep: 60/20 = 3 RPM
#define COGGING_CALIB_HI_RPM            30.0f   // RPM#2 sweep speed
#define COGGING_CALIB_ULTRA_RPM         100.0f  // RPM#3 sweep speed
#define COGGING_CALIB_DFT_HARMONICS     128     // Harmonics analyzed during DFT
#define COGGING_HARMONICS_COUNT         20      // Stored in each map
#define COGGING_DFT_USE_IQ_CMD                  // Use iq_cmd (residual) for DFT, not raw ADC

// target_constants.h (all targets)
#define COGGING_TABLE_FLASH_START_ADDRESS 0x08010000
#define COGGING_TABLE_SIZE               (COGGING_HARMONICS_COUNT * 12)  // 240
#define COGGING_DRIVER_COUNT             3
#define COGGING_TABLES_PER_DRIVER        3
#define MAX_COGGING_TABLES               9
```

---

## 6. Position Precision

### Float Mantissa Protection

A 32-bit `float` has 24 bits of mantissa. High-resolution encoders (e.g., 22-bit BISS-C) use most of this for a single fraction of a turn. Accumulating full turns would push fractional data out of the mantissa window.

**The fix**: `getFilteredPosition()` performs integer modulo `pos % cpr` *before* float conversion, guaranteeing 100% of the encoder's raw fractional resolution indefinitely. This is used exclusively for the Fourier series phase lookup and waveform reconstruction.

### Polymorphic Encoder Routing

```cpp
Encoder* activeEnc = usingExternalEncoder() ? drvEncoder.get() : (Encoder*)this;
```

Both `TMC4671` and external drivers inherit from `Encoder`, so `activeEnc->getPos_f()` routes to the active hardware while inheriting the mantissa protection.

---

## 7. Diagnostics

### Electrical Diagnostic (`COGGING_CALIB_ENABLE_ID_DIAG`)
Analyzes the Id (flux) axis. Significant H3 or H6 energy indicates phase imbalance or partial winding short.

### Mechanical Diagnostic (Eccentricity)
High H1 magnitude on the Iq axis signals rotor eccentricity, bent shaft, or encoder misalignment.

### Harmonic Anti-Cogging
The system scans 128 harmonics and selects the top 20 peaks. Orders 1–10 are excluded to capture magnetic detent while ignoring gravitational imbalance from asymmetric wheels.

### Live Debugging
A global `g_tmc4671_cogging_debug` struct provides ST-Link live-inspection of the current calibration phase, PID rate, position error, `iqPid`, `iqFriction`, `iqInertia`, `iqCompensation`, `iqApplied`, and per-quarter max error.

---

## 8. Configurator Integration

### Multi-RPM Blend Tab

The **Manual Tuning** dialog (Scale & Phase Advance Curves) includes a **Multi-RPM Blend** tab with:

- **RPM#2 crossover** spinbox (editingFinished → `send_value coggingBlendHigh`)
- **RPM#3 crossover** spinbox (editingFinished → `send_value coggingBlendUltra`)
- **Clear maps** button (sends `coggingClearHi` GET — cannot fire from spinbox Enter due to `setAutoDefault(False)`)
- **Overlay chart**: low (blue), RPM#2 (red), RPM#3 (orange) — one revolution of each reconstructed waveform
- **Validity status**: shows ✓/✗ for both RPM#2 and RPM#3

### Scale & Phase Curve Editors

24 RPM breakpoints: `{0, 5, 7, 10, 12, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200, 225, 256}`. Each point is a spinbox; edits push to firmware immediately. Live RPM dot tracks current velocity on the chart.

### Harmonic Editor Tab

Slider + spinbox for shaping factor (−1.0 to +1.0), phase trim (−180° to +180°), and multiplier (1–31). Chart previews original vs shaped waveform over one revolution.

---

## 9. Design Rationale

### Why Three Maps Instead of One?

A single DFT map captured at 3 RPM is accurate at low speed but degrades as speed increases because the closed-loop torque path introduces:

1. **Amplitude attenuation**: the TMC4671 current loop has finite bandwidth; at higher electrical frequencies the commanded $i_q$ amplitude rolls off.
2. **Phase lag**: the main loop read→compute→command latency, TMC4671 internal current-loop delay, and encoder processing delay produce a phase lag proportional to $\omega \cdot \tau$ per harmonic.

A single map cannot compensate for both simultaneously across a wide speed range. The multi-map approach measures the *net* distortion at each speed band and replays it — the map captured at speed $X$ cancels perfectly at speed $X$ and blends smoothly in between.

### Why Blend in Torque Domain?

The three maps may contain different top-20 harmonic orders (a harmonic that dominates at 3 RPM may be buried at 100 RPM, and vice versa). Matching orders across tables for phasor-domain blending is lossy and fiddly. Waveform-domain blending is mathematically equivalent and handles arbitrary harmonic sets.

### Why the Clear Button Uses `setAutoDefault(False)`?

In Qt `QDialog`, any `QPushButton` with `autoDefault=True` (the default) fires its `clicked()` signal when Enter is pressed anywhere in the dialog. Without `setAutoDefault(False)`, pressing Enter in the crossover spinbox would simultaneously send the crossover SET **and** fire the Clear button → clear the hi/ultra maps → maps vanish.
