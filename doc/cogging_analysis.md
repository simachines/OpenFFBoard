# Anti-Cogging Calibration Guide — Continuous DFT-128 Method

This document describes the harmonic-based anti-cogging calibration system for the OpenFFBoard TMC4671 driver.
The system uses cosine/sine accumulation at 128 harmonic orders over one mechanical revolution to extract
the detent torque profile, then replays it as a Fourier-series feedforward to cancel cogging in real time.

---

## 1. Quickstart — Running a Calibration

1. Open the **Configurator**, select your TMC driver tab.
2. Click **"Cogging Calibration"** (button below the state indicator). This opens the multi-tab dialog.
3. In the **"Cogging Calibration"** tab (first tab):
   - Set the **number of RPM profiles** (1–5, default 3 from firmware).
   - PER-PROFILE: set the target RPM (defaults: 3/10/20 RPM) and number of DFT iterations.  
     *One iteration at low RPM is usually enough for a clean map.*
   - Check **"Auto Velocity PID Tune"** to let the firmware find the optimal speed-loop P gain for each profile.
   - Keep **"Inertia Acceleration Correction"** OFF unless you have high inertia and see noisy results.  
     *(When ON, the DFT subtracts $J \cdot \alpha$ from the signal. Requires a valid $J$ from SysId.)*
   - Keep **"Friction Feedforward"** OFF unless you need to cancel viscous drag during the sweep.
4. Press **"Start Cogging Calibration"**.
5. Watch the calibration log. The motor will:
   - Break static friction, then measure $J$ (inertia pulse) and $B$ (30 RPM steady-state).
   - Compute IMC-optimal PID gains and run a validation rotation.
   - For each RPM profile: auto-tune P (if enabled), run CW+CCW DFT sweeps, broadcast the harmonic table.
   - Run a **verification pass** — spins with feedforward active and prints residual harmonic amplitudes.
6. The **Harmonic Editor** tab (third tab) populates with per-harmonic magnitude spinboxes after each profile completes. You can edit magnitudes and press **"Apply to Firmware"** to push changes live.

---

## 2. End-to-End Data Flow

### 2.1 Encoder Counts → Normalized Position

```
raw_encoder_counts % CPR → remainder → (float)remainder / CPR → pos_f ∈ [0.0, 1.0)
```

Integer modulo *before* float conversion preserves the encoder's full fractional resolution — critical for high-CPR encoders (22-bit BISS-C, etc.) where float mantissa would otherwise lose precision after accumulating full turns.

### 2.2 DFT Accumulation (Recursive Chebyshev)

During each CW or CCW sweep, at every Nth loop iteration (decimated by `dft_decimation_ratio = 2`):

```cpp
float s1, c1;
arm_sin_cos_f32(pos_f * 360.0f, &s1, &c1);     // sin(θ), cos(θ) — just once per sample

float cur_s = s1, cur_c = c1;
for (int k = 1; k < 128; k++) {
    iq_acc_cos[k] += (iq * cur_c);               // accumulate Re
    iq_acc_sin[k] += (iq * cur_s);               // accumulate Im

    // Angle-addition formulas for the next harmonic — no extra trig calls
    float next_c = cur_c * c1 - cur_s * s1;       // cos((k+1)θ) = cos(kθ)·cos(θ) − sin(kθ)·sin(θ)
    float next_s = cur_c * s1 + cur_s * c1;       // sin((k+1)θ) = sin(kθ)·cos(θ) + cos(kθ)·sin(θ)
    cur_c = next_c; cur_s = next_s;
}
```

The DFT signal `iq` is the **PID residual torque** (velocity-loop output + friction FF, optionally minus inertia). When `COGGING_DFT_USE_IQ_CMD` is defined (the default), it's the command-units PID output — dimensionally uniform with the compensation table.

### 2.3 Normalization → Magnitude & Phase

```cpp
float norm = 2.0f / dir_samples;        // DFT normalization
float re = iq_acc_cos[k] * norm;
float im = iq_acc_sin[k] * norm;
float mag = sqrtf(re*re + im*im);       // amplitude in torque-command units
float phase = atan2f(re, im);           // phase in radians
```

### 2.4 CW+CCW Averaging (Complex Vector Method)

CW and CCW sweeps produce independent `[mag, phase]` pairs. Directly averaging phases can produce wrong results near ±π due to wrap-around. The method uses complex vector averaging for phases:

```cpp
// Average magnitude (cancels AC-neutral friction bias)
float avg_mag = (cw_harms[k].mag + ccw_harms[k].mag) / 2.0f;

// Convert phases to complex vectors, average, extract combined phase
// (avoids wrap-around bugs near ±π)
float cw_re = cosf(cw_harms[k].phase);
float cw_im = sinf(cw_harms[k].phase);
float ccw_re = cosf(ccw_harms[k].phase);
float ccw_im = sinf(ccw_harms[k].phase);
float avg_re = (cw_re + ccw_re) / 2.0f;
float avg_im = (cw_im + ccw_im) / 2.0f;
float avg_phase = atan2f(avg_im, avg_re);
```

This cancels friction bias (AC-neutral) — friction adds equally in both directions, so averaging cancels it out. It also cancels PID tracking-lag delta (phase shifts in opposite directions for CW vs CCW), leaving only the pure cogging component.

### 2.4b Combined DFT Harmonics vs Combined Bins

The system produces **two different combined datasets** from the CW and CCW sweeps:

| Dataset | How it's made | Purpose |
|---|---|---|
| **Combined DFT harmonics** (`cogging_harmonics[]`) | Average CW & CCW DFT magnitudes; complex-vector average of phases. Top 20 by magnitude. | FF mode 0 (harmonic sum), configurator graph |
| **Combined bins** (`cogging_bins_combined[720]`) | Simple spatial average: `(cw_bins[b] + ccw_bins[b]) * 0.5f` | FF mode 1 (bin LUT) |

The configurator's Harmonic Editor graph reconstructs a waveform from the **combined DFT harmonics** only — summing `amp × sin(order × θ + phase)` for each harmonic. This produces a smooth, sinusoidal curve. The bins (raw spatial torque samples) are noisier and include higher-frequency content that the top-20 DFT truncates.

### 2.5 Top-N Selection → Compensation Table

The top 20 harmonic orders by combined magnitude become `cogging_harmonics[]`.

### 2.6 Runtime Feedforward

In `TMC4671::turn()`, the harmonic table is summed as a Fourier series each time torque is commanded:

```cpp
float compensation = 0;
for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
    if (cogging_harmonics[i].amplitude > 0.0f) {
        compensation += cogging_harmonics[i].amplitude
            * arm_sin_f32(angle_rad * cogging_harmonics[i].order + cogging_harmonics[i].phase);
    }
}
totalPower += cogging_scale * compensation;
```

---

## 3. System Identification (SysId)

Before DFT, the calibration measures the motor's physical parameters to compute IMC-optimal PID gains.

| Step | What | How |
|:---|:---|:---|
| Breakout torque | Min torque to move | Incremental torque ramps until position moves ≥0.005 turns |
| Inertia $J$ | $J = (\tau \cdot \Delta t^2) / (2 \cdot \Delta\theta)$ × 100 | 150 ms constant-torque pulse |
| Viscous friction $B$ | $B = (\tau_{\text{avg}} / \omega) \times 100$ | 2 s CW rotation at 30 RPM |
| IMC bandwidth | $f_{bw} = \text{clamp}(16.5 - 0.0047J,\ 6,\ 15)$ Hz | Degrades with inertia |

All values are in abstract TMC torque-command units. J and B are scaled ×100 to stay in a numerically stable range for the CMSIS PID computations.

---

## 4. P-Gain Auto-Tuning (per RPM profile)

When "Auto Velocity PID Tune" is enabled, the firmware runs a **trapezoidal velocity sweep** (accel → cruise → decel) with zero I and D to find the optimal P that minimizes peak-to-peak position error without saturation:

- Starts at 75% of the IMC-derived baseline Kp (or profile 0's stored Kp for higher profiles).
- Sweeps up (×1.25 per step) and down, alternating direction to bracket the minimum.
- If the torque command saturates (clamps), backs off and re-tests.
- Reports `Selected Kp:XXXX (Optimal, Lowest P2P:X.XX°)`.

The auto-tuned P is stored in `cogging_calib_pidP[rpm_profile]` for the DFT sweeps.

---

## 5. Multi-RPM Profile System

Up to 5 configurable RPM profiles (`COGGING_MAX_CALIB_PROFILES = 5`), each with independent
RPM target, DFT iterations, and velocity PID settings. The first 3 profiles have dedicated
flash storage tables; profiles 4+ reuse the last table for PID storage only:

| Profile | Default RPM | DFT Iters | Flash Index | RAM Table |
|:---|:---|:---|:---|:---|
| Profile 1 (Low) | 3.0 | 1 | `drv-1 + 0×3` | `cogging_harmonics` |
| Profile 2 (Mid) | 10.0 | 1 | `drv-1 + 1×3` | `cogging_harmonics_rpm2` |
| Profile 3 (High) | 20.0 | 1 | `drv-1 + 2×3` | `cogging_harmonics_rpm3` |
| Profile 4+ | user-set | user-set | N/A (PID only) | N/A |

### Runtime blending

*Requires `COGGING_DISABLE_BLEND` to be commented out (currently active — blending disabled).*

When enabled, `blendHarmonicTables(measured_rpm, blended)` interpolates between adjacent
tables at the **harmonic level** — matching orders between tables, then linearly interpolating
amplitude and phase with unwrapping:

```cpp
float t = (rpm - blend_rpm1) / (blend_rpm2 - blend_rpm1);
w_lo = 1.0f - t;  w_hi = t;
// For each harmonic order found in tab_a:
out_table[i].amplitude = amp_a * w_lo + amp_b * w_hi;
out_table[i].phase = ph_a * w_lo + ph_b * w_hi;
```

Below `blend_rpm1` → 100% low; above highest valid → 100% that table.

### Per-profile DFT targeting

Each profile's DFT measures into a dedicated `active_tbl` pointer:

```cpp
Harmonic* active_tbl = this->cogging_harmonics;
if (rpm_profile == 1) active_tbl = this->cogging_harmonics_rpm2;
else if (rpm_profile >= 2) active_tbl = this->cogging_harmonics_rpm3;
```

The routine is identical for every profile: `memset(active_tbl, 0)` at start, empty on
iteration 0 → measures full cogging fresh, phasor-add refines on later iterations.

---

## 6. Verification Pass

After each profile's DFT completes, the firmware runs a **CW + CCW verification pass**: one revolution in each direction with the finalized feedforward active, then extracts and prints the residual harmonics:

```
VERIFY CW residual (order : amplitude : phase_rad):
1 : 115.7 : -0.0779
2 : 73.9 : 1.3866
...
VERIFY CCW residual (order : amplitude : phase_rad):
1 : 120.2 : -3.0138
2 : 76.0 : -1.3629
...
```

This shows exactly what cogging remains after compensation. Use the Harmonic Editor to tweak magnitudes and re-verify.

---

## 7. Configurator — Harmonic Editor Tab

Replaces the old waveshaping controls with direct per-harmonic magnitude editing:

- **RPM profile selector**: Switch between calibrated RPM maps.
- **Chart view toggle**: Waveform (angle domain) or Harmonic Magnitudes (bar chart).
- **Per-harmonic magnitude spinboxes**: Edit the amplitude of each harmonic. Amplitudes are in abstract torque-command units (same as the DFT output).
- **"Apply to Firmware" button**: Sends the edited magnitudes to the MCU via `coggingH3` setat commands (adr=3 clear table, adr=4 set amplitude+order, adr=5 set phase).
- **Live position dot + angle readout**: Red dot on the waveform chart tracks the actual rotor angle, with a numeric "Angle: XXX.X°" label.
- **CW/CCW overlay checkbox**: Shows raw direction-specific waveforms for comparison.
- **Download/Copy/Load/Clear buttons**: Export/import harmonic data as text files.

---

## 8. Scale Curve & Phase Advance

24-point RPM breakpoints: `{3, 5, 7, 10, 12, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200, 225, 256}`.

- **Scale curve**: Interpolates `cogging_scale` by measured RPM. Compensates for current-loop amplitude rolloff at higher speeds.
- **Phase advance**: Shifts the cogging lookup position by `sign(rpm) * adv_deg / 360` before the Fourier sum. Compensates for loop latency phase lag.

Both curves are stored in EEPROM and editable from the Configurator's Scale Curve / Phase Advance tabs. Live RPM dot shows current velocity on the chart.

---

## 9. ST-Link Debug Watch

The global struct `g_tmc4671_cogging_debug` provides live inspection during calibration:

| Field | Description |
|:---|:---|
| `phase` | Current calibration phase (SysId, Validation, Acquisition, etc.) |
| `positionErrorDeg` | Tracking error × 36000 (×10 scaled) |
| `angle` | Actual rotor angle 0–360° |
| `iqCmd` | PID + friction torque command |
| `iqCompensation` | Cogging feedforward torque |
| `Appliediq` | Total applied torque (iqCmd + compensation) |
| `pidExecRate` | Microseconds between captureDebug calls (inverse of PID rate) |

---

## 2. Runtime Compensation

### 2.1 Feed-Forward Source Selection

The `cogging_ff_mode` parameter selects the compensation source:

| Mode | Source | Description |
| :--- | :--- | :--- |
| 0 | Harmonic sum (default) | Fourier sum from `cogging_harmonics[]` table |
| 1 | Combined bin LUT | `cogging_bins_combined[]` — friction-free, pure cogging |
| 2 | Per-direction bin LUT | `cw_bins[]` or `ccw_bins[]` — includes friction asymmetry |

### 2.2 Multi-RPM Harmonic Blending

*Requires `COGGING_DISABLE_BLEND` to be commented out (currently active — disabled).*

When enabled, `blendHarmonicTables(measured_rpm, blended)` interpolates between adjacent
harmonic tables at the **harmonic level** (not torque-domain). Orders present in only one
table of a blend pair are dropped at the blend midpoint:

| RPM range | Compensation |
| :--- | :--- |
| $\text{rpm} \le \text{blend\_rpm1}$ | 100% low table (`cogging_harmonics`) |
| $\text{blend\_rpm1} < \text{rpm} \le \text{blend\_rpm2}$ | Harmonic lerp low ↔ rpm2 |
| $\text{blend\_rpm2} < \text{rpm} \le \text{blend\_rpm3}$ | Harmonic lerp rpm2 ↔ rpm3 |
| $\text{rpm} > \text{blend\_rpm3}$ | 100% highest valid table |

The crossover points default to the calibration sweep speeds.

### 2.3 Velocity-Based Phase Advance

A position offset proportional to RPM compensates for high-speed loop latency. Stored as a 24-point curve (degrees vs RPM) in EEPROM. Applied before waveform lookup:

$$\theta' = \theta + \text{sign}(\omega) \cdot \frac{\text{adv}(\text{rpm})}{360}$$

### 2.4 Speed-Dependent Scale Curve

A 24-point curve interpolates `cogging_scale` by measured RPM to counteract current-loop amplitude rolloff at higher speeds. Stored in EEPROM alongside the phase-advance curve.

### 2.5 Harmonic Waveshaping

A shaping term applied to the dominant harmonic to thin peaks / steepen slopes:

$$T_{\text{shaped}} = T_{\text{comp}} - s \cdot A_{\text{dom}} \cdot \sin\!\big(m \cdot (k_{\text{dom}} \cdot \theta + \phi_{\text{dom}}) + \phi_{\text{trim}}\big)$$

- $s$: shaping factor (−1.0 to +1.0), positive thins peaks
- $m$: harmonic multiplier (default 3)
- $\phi_{\text{trim}}$: extra phase offset

### 2.6 Full Runtime Formula

$$T_{\text{comp}}(\theta, \omega) = \text{scale}(\omega) \cdot \left[ \text{blend}\!\big(T_1(\theta'),\, T_2(\theta'),\, T_3(\theta'),\, \omega\big) - \text{shape}(T_{\text{dom}}) \right]$$

$$T_{\text{final}} = T_{\text{requested}} + T_{\text{comp}}$$

---

## 3. Storage Architecture

### 3.1 Flash Cogging Region

Three maps per TMC driver, stored in a dedicated flash sector (sector 4 at `0x08010000`):

| Slot | Contents |
| :--- | :--- |
| 0–2 | Low-RPM maps (drv0, drv1, drv2) — `cogging_harmonics` |
| 3–5 | Mid-RPM maps (drv0, drv1, drv2) — `cogging_harmonics_rpm2` |
| 6–8 | High-RPM maps (drv0, drv1, drv2) — `cogging_harmonics_rpm3` |

Each slot: `COGGING_TABLE_SIZE = COGGING_HARMONICS_COUNT × 12 = 240` bytes (20 harmonics × {float amp, float phase, uint16 order}).

Static buffer in `flash_helpers.cpp`: `cogging_flash_buffer[4096]` (9 × 240 = 2160 bytes, rounded up).

Read-modify-write via `Flash_WriteCoggingTable()` preserves other tables during writes.

### 3.2 EEPROM Settings

Per-driver settings persisted in the EEPROM emulation region:

| Address Range | Content |
| :--- | :--- |
| `0x32D–0x32F` | Cogging enable, scale, dynamic offset |
| `0x420–0x4AF` | 24-pt scale curve + 24-pt phase advance curve (all 3 drivers) |
| `0x4B0–0x4B8` | H3 waveshaping per driver (`h3Shaping`, `h3PhaseTrim`, `h3Mult`) |
| `0x4B9–0x4BE` | Blend RPM#2 crossover + valid flag per driver |
| `0x4BF–0x4C4` | Blend RPM#3 crossover + valid flag per driver |

`NB_OF_VAR = 399`.

### 3.3 RAM Footprint

| Component | Size | Lifecycle |
| :--- | :--- | :--- |
| `cogging_harmonics[]` (low) | 240 B | Permanent |
| `cogging_harmonics_rpm2[]` | 240 B | Permanent |
| `cogging_harmonics_rpm3[]` | 240 B | Permanent |
| `cw_store[]` / `ccw_store[]` | 480 B | Permanent |
| `cw_bins[720]` / `ccw_bins[720]` | 5.6 KB | Permanent |
| `cogging_bins_combined[720]` | 2.8 KB | Permanent |
| Scale curve values | 96 B | Permanent |
| Phase advance curve values | 96 B | Permanent |
| DFT accumulators (128 harmonics) | ~2 KB | Calibration only (heap) |
| Spatial bin accumulators (720 bins) | ~3 KB | Calibration only (heap) |
| **Total permanent** | ~10 KB | — |

---

## 4. Commands Reference

### Calibration Commands

| Command | Access | Description |
| :--- | :--- | :--- |
| `calibrateCogging` | GET | Start calibration |
| `cogging` | GET/SET | Enable/disable (1/0) |
| `coggingScale` | GET/SET | Global scale (×10000) |
| `coggingFFMode` | GET/SET | FF source: 0=harmonics, 1=combined bins, 2=per-direction bins |
| `coggingBins` | GETADR | Spatial bin snapshots (adr 0=CW, 1=CCW, 2=verCW, 3=verCCW, 4=verCWharm, 5=verCCWharm) |
| `coggingHarmonics` | GET | Harmonic table: `order:amp:phase,…` (adr 0=low, 1=rpm2, 2=rpm3) |
| `coggingCwCcw` | GET | Raw CW/CCW harmonics (`CW:…|CCW:…`) |
| `coggingH3` | GET/SETADR | Waveshaping: adr 3=clear, 4=set amp+order, 5=set phase |
| `coggingSave` | GET | Save cogging table to flash |

### Multi-RPM Commands

| Command | Access | Format | Description |
| :--- | :--- | :--- | :--- |
| `coggingBlendRpm2` | GET/SET | RPM ×10 | RPM#2 blend crossover |
| `coggingBlendRpm3` | GET/SET | RPM ×10 | RPM#3 blend crossover |
| `coggingRpm2Valid` | GET | `0` or `1` | RPM#2 map validity |
| `coggingRpm3Valid` | GET | `0` or `1` | RPM#3 map validity |

### Curve Commands

| Command | Access | Description |
| :--- | :--- | :--- |
| `scaleCurve` | GET/SETADR | 24-pt scale curve (value ×1000) |
| `phaseAdvCurve` | GET/SETADR | 24-pt phase advance (deg ×100) |

### Per-Profile Calibration Commands

| Command | Access | Description |
| :--- | :--- | :--- |
| `coggingCalibCount` | GET/SET | Number of RPM profiles (1–5) |
| `coggingCalibRPM` | GETADR/SETADR | RPM ×10 per profile |
| `coggingCalibIters` | GETADR/SETADR | DFT iterations per profile |
| `coggingCalibPidP/I/D` | GETADR/SETADR | Manual velocity PID per profile |
| `coggingCalibAutoPid` | GET/SET | Auto PID tune (1=auto, 0=manual) |
| `coggingCalibInertiaCorr` | GET/SET | Inertia acceleration correction (1=on) |
| `coggingCalibFrictionFF` | GET/SET | Friction feedforward during DFT (1=on) |

---

## 5. Configuration Constants

```c
// TMC4671.h
#define COGGING_CALIB_TIME_PER_REV_S    20      // Low-RPM sweep: 60/20 = 3 RPM
#define COGGING_CALIB_DFT_HARMONICS     128     // Harmonics analyzed during DFT
#define COGGING_DFT_BIN_COUNT           720     // Spatial bins for noise-robust DFT (0.5° per bin)
#define COGGING_HARMONICS_COUNT         20      // Stored in each map
#define COGGING_MAX_CALIB_PROFILES      5       // Max configurable RPM profiles
#define COGGING_DFT_USE_IQ_CMD                  // Use iq_cmd (residual) for DFT, not raw ADC
// #define COGGING_DISABLE_BLEND                  // Uncomment to enable multi-RPM blending
// #define COGGING_PHASE_SHIFT_MULTIRPM          // Uncomment for 23-point scale/phase curves

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
The system scans 128 harmonics (`k = 1..127`) and selects the top 20 peaks by magnitude.
Order 1 is included — it captures residual mechanical imbalance or eccentricity.
Orders beyond ~20 typically have negligible amplitude on most motors, but the full
128-order DFT ensures they won't alias into the selected band.

### Live Debugging
A global `g_tmc4671_cogging_debug` struct provides ST-Link live-inspection of the current calibration phase, PID rate, position error, `iqPid`, `iqFriction`, `iqInertia`, `iqCompensation`, `iqApplied`, and per-quarter max error.

---

## 8. Configurator Integration

### Multi-RPM Blend Tab

The **Manual Tuning** dialog (Scale & Phase Advance Curves) includes a **Multi-RPM Blend** tab with:

- **RPM#2 crossover** spinbox (editingFinished → `send_value coggingBlendRpm2`)
- **RPM#3 crossover** spinbox (editingFinished → `send_value coggingBlendRpm3`)
- **Clear maps** button (sends `coggingClearHi` GET — cannot fire from spinbox Enter due to `setAutoDefault(False)`)
- **Overlay chart**: profile 1 (blue), RPM#2 (red), RPM#3 (orange) — one revolution of each reconstructed waveform
- **Validity status**: shows ✓/✗ for both RPM#2 and RPM#3

### Harmonic Editor Tab

- **RPM profile selector**: Switch between calibrated RPM maps.
- **Chart view toggle**: Waveform (angle domain) or Harmonic Magnitudes (bar chart).
- **Per-harmonic magnitude spinboxes**: Edit the amplitude of each harmonic.
- **"Apply to Firmware" button**: Sends edited magnitudes to the MCU via `coggingH3` setat.
- **Live position dot + angle readout**: Red dot tracks actual rotor angle.
- **CW/CCW overlay checkbox**: Shows raw direction-specific waveforms.
- **Download/Copy/Load/Clear buttons**: Export/import harmonic data as text files.

### Scale & Phase Curve Editors

24 RPM breakpoints: `{3, 5, 7, 10, 12, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200, 225, 256}`. Each point is a spinbox; edits push to firmware immediately. Live RPM dot tracks current velocity on the chart.

---

## 9. Design Rationale

### Why Three Maps Instead of One?

A single DFT map captured at low RPM is accurate at low speed but degrades as speed increases because the closed-loop torque path introduces:

1. **Amplitude attenuation**: the TMC4671 current loop has finite bandwidth; at higher electrical frequencies the commanded $i_q$ amplitude rolls off.
2. **Phase lag**: the main loop read→compute→command latency, TMC4671 internal current-loop delay, and encoder processing delay produce a phase lag proportional to $\omega \cdot \tau$ per harmonic.

A single map cannot compensate for both simultaneously across a wide speed range. The multi-map approach measures the *net* distortion at each speed band and replays it — the map captured at speed $X$ cancels perfectly at speed $X$ and blends smoothly in between.

### Why Blend at the Harmonic Level?

The current `blendHarmonicTables()` blends at the **harmonic level** (matching orders between adjacent tables, lerping amplitude and phase). This is simpler than waveform-domain blending and works well when the dominant harmonics are consistent across RPM bands. Orders present in only one table are dropped at the blend midpoint — if this proves lossy, blending could be changed to the waveform/compensation domain instead.

### Why the Clear Button Uses `setAutoDefault(False)`?

In Qt `QDialog`, any `QPushButton` with `autoDefault=True` (the default) fires its `clicked()` signal when Enter is pressed anywhere in the dialog. Without `setAutoDefault(False)`, pressing Enter in the crossover spinbox would simultaneously send the crossover SET **and** fire the Clear button → clear the hi/ultra maps → maps vanish.
