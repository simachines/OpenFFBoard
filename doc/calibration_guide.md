# OpenFFBoard Anti-Cogging Calibration — Step-by-Step Guide

> **Active defines:** `COGGING_TABLE_FLASH_START_ADDRESS`, `COGGING_DFT_USE_IQ_CMD`, `COGGING_DISABLE_BLEND`
>
> **Optional defines:** `COGGING_DISABLE_SCALE_CURVE`, `COGGING_PHASE_SHIFT_MULTIRPM`

---

## Overview

The calibration measures motor cogging torque and computes a harmonic Fourier feedforward table.  
It runs **up to 5 configurable RPM profiles** (default 3 profiles at 3/10/20 RPM), each with:

1. **P-gain auto-tuning** — finds optimal velocity-loop Kp via trapezoidal sweeps
2. **DFT acquisition** — CW/CCW constant-speed revolutions while recording IQ torque
3. **CW+CCW harmonic averaging** — cancels friction bias and PID tracking lag

Total output:

- `cogging_harmonics[20]` — spatial anti-cogging Fourier table (per profile, stored in flash)
- `scale_curve_values[24]` — attenuation vs RPM (unless `COGGING_DISABLE_SCALE_CURVE`)
- `phase_advance_curve_values[24]` — phase shift in mechanical degrees vs RPM (unless `COGGING_DISABLE_SCALE_CURVE`)
- `cw_bins[720]` / `ccw_bins[720]` — raw spatial bin snapshots (per direction, for configurator readout)
- `cogging_bins_combined[720]` — averaged CW/CCW bin LUT (used by FF mode 1 at runtime)

---

## Compile-Time Feature Switches

| Define | Effect when defined | Current state |
|---|---|---|
| `COGGING_PHASE_SHIFT_MULTIRPM` | Tests all 23 fixed RPM breakpoints; extracts phase lag per RPM via master harmonic tracking | ✗ Commented out |
| `COGGING_DISABLE_SCALE_CURVE` | **No scale/phase curves** — `cogging_scale`=1.0 at all RPMs | ✗ Commented out (curves active) |
| `COGGING_DISABLE_BLEND` | Uses only `cogging_harmonics` base table at all RPMs | ✓ Active |
| `COGGING_DFT_USE_IQ_CMD` | DFT uses `iq_cmd` (PID+friction effort) instead of raw ADC `actual_iq_raw` | ✓ Active |

### `COGGING_DISABLE_BLEND` (currently active)

When active (the default), the runtime feedforward always reads from the base `cogging_harmonics[]` table.
Multi-RPM blending between the three stored tables (low/mid/high) is disabled.

When commented out, `blendHarmonicTables(measured_rpm, blended)` blends adjacent tables at the
**harmonic level** (matches orders, lerps amplitude and phase with unwrapping).

### `COGGING_DISABLE_SCALE_CURVE` (currently commented out)

Scale curve and phase advance are **active** — `turn()` applies RPM-dependent amplitude scaling
and phase advance at runtime.

When defined, the following are ALL disabled:

- `scale_curve_valid` — never set to true (calibration or loadFlash)
- `phase_adv_curve_valid` — never set to true
- `scale_curve_values[]` writes — skipped
- Load-time `scale_curve_valid` restoration — skipped
- Fill-in interpolation for scale values — skipped

**Runtime effect when disabled**: `turn()` uses `cogging_scale` (1.0) directly, no phase-advance.
**Runtime effect when active** (current): RPM-dependent amplitude scaling via the scale curve.

### `COGGING_PHASE_SHIFT_MULTIRPM` (currently commented out)

When active, tests all 23 fixed RPM breakpoints from `scale_curve_rpm_points[]` and extracts
phase lag per RPM via master harmonic tracking. Populates both `scale_curve_values[]` and
`phase_advance_curve_values[]` from the calibration data.

When commented out (current), the calibration uses only the user-configured `cogging_calib_count`
profiles (default 3: 3/10/20 RPM). Scale and phase-advance curves can still be tuned manually
from the configurator.

---

## STEP 0 — Setup

### Variables initialised

| Variable | Type | Default | Purpose |
|---|---|---|---|
| `max_test_torque` | `float` | `bangInitPower × 0.8` (~1600) | Torque limit for sweeps |
| `J` | `float` | Measured | Motor inertia (TMC abstract units, physical × 100) |
| `B` | `float` | Measured | Viscous friction |
| `dynamic_friction` | `float` | Measured | Coulomb friction breakout torque |
| `cogging_warmup_ms` | `uint32_t` | `max(1500, J/100×rpm + 1000)`, capped 8000 | Velocity ramp-up duration |
| `cogging_calib_count` | `uint8_t` | `1` (default 3 from init) | Number of RPM profiles (up to `COGGING_MAX_CALIB_PROFILES` = 5) |
| `cogging_calib_rpm[]` | `float[5]` | `{3.0, 10.0, 20.0, 0, 0}` | RPM targets per profile |
| `cogging_calib_iters[]` | `uint16_t[5]` | `{1, 1, 1, 0, 0}` | DFT iterations per profile |

### Curve arrays zeroed

```cpp
memset(scale_curve_values, 0, sizeof(scale_curve_values));
memset(phase_advance_curve_values, 0, sizeof(phase_advance_curve_values));
scale_curve_values[0] = 1.0f;
phase_advance_curve_values[0] = 0.0f;
```

---

## STEP 1 — Encoder Identification

Determines encoder resolution category:

| CPR range | Decimation | Penalty | `calib_rpm` | Label |
|---|---|---|---|---|
| < 20,000 | 8 | 0.2 | 12.0 | Low-Res |
| 20k–50k | 4 | 0.5 | 6.0 | Medium-Res |
| ≥ 50,000 | 1 | 1.0 | `60 / COGGING_CALIB_TIME_PER_REV_S` (3 RPM) | High-Res |

---

## STEP 2 — Physical System Identification (J & B)

### 2.1 Break static friction

```cpp
// Increment torque in 100-unit steps until motor moves >0.005 turns
while (!friction_broken && tuning_torque < max_test_torque) {
    applySafeTorque(tuning_torque);
    Delay(BREAKOUT_STEP_MS); // 50ms
    if (fabs(error) > 0.005f) friction_broken = true;
    else tuning_torque += 100.0f;
}
```

### 2.2 Measure Inertia (J)

```cpp
// Apply j_torque pulse for SYSID_J_PULSE_MS (150ms)
// J = (Torque × dt²) / (2 × delta_pos_rad) × 100
J = ((j_torque * dt_j * dt_j) / (2.0f * d_pos_rad)) * 100.0f;
```

### 2.3 Measure Friction (B)

```cpp
// Constant velocity sweep at 30 RPM for SYSID_B_DURATION_MS (2000ms)
// B = (average_torque / target_vel_rad) × 100
B = (b_sum_torque / (float)b_samples) / b_target_vel_rad * 100.0f;
```

### 2.4 IMC Pole Placement (baseline Kp/Ki)

```cpp
float f_bw = clip(16.5f - 0.0047f*J, 6.0f, 15.0f); // bandwidth Hz
float wn = 2π × f_bw;
imc_kp = 2×ζ×wn×J - B;  // ζ=1.0 for critical damping
imc_ki = wn² × J × ki_scale;
```

### Dynamic warmup scaling

```cpp
// After J is known:
cogging_warmup_ms = max(1500, J/100.0f × calib_rpm + 1000), capped at 8000;
```

---

## STEP 3 — Multi-RPM Calibration Loop

```cpp
for (uint8_t rpm_profile = 0;
     rpm_profile < this->cogging_calib_count && !emergency && hasPower();
     rpm_profile++) {

    calib_rpm = this->cogging_calib_rpm[rpm_profile];
    if (calib_rpm <= 0.0f) calib_rpm = 60.0f / (float)COGGING_CALIB_TIME_PER_REV_S;
    const uint8_t MAX_DFT_ITERATIONS = this->cogging_calib_iters[rpm_profile];
```

### 3a — Per-Profile PID Loading

```cpp
uint8_t pid_src = rpm_profile;
uint32_t pidP = this->cogging_calib_pidP[pid_src];
uint32_t pidI = this->cogging_calib_pidI[pid_src];
uint32_t pidD = this->cogging_calib_pidD[pid_src];
// If all zeros (unconfigured slot), copy from slot 0 (IMC baseline):
if (pid_src > 0 && pidP == 0 && pidI == 0 && pidD == 0) {
    pidP = this->cogging_calib_pidP[0];
    pidI = this->cogging_calib_pidI[0];
    pidD = this->cogging_calib_pidD[0];
}
this->coggingSpeedP = (float)pidP;
```

---

## STEP 3b — P-Gain Auto-Tuner (per profile)

### Trajectory Setup

```cpp
float j_phys = J / 100.0f;        // physical inertia
float max_accel_turns_s2 = (max_test_torque × 0.25) / j_phys / 2π;
if (max_accel_turns_s2 < 1.0f) max_accel_turns_s2 = 1.0f;

float target_vel_turns = calib_rpm / 60.0f;
float ramp_dist = max(v²/2a, 0.25);          // min 0.25 turns ramp
float cruise_dist = max(v×0.6, 0.35);        // min 0.35 turns cruise
float total_dist = ramp×2 + cruise;
// TIME CAP: limit total to ~3s (prevents 40s sweeps at 3 RPM)
float max_total = target_vel_turns × 3.0;
if (total_dist > max_total) total_dist = max_total;
if (total_dist < 0.5) total_dist = 0.5;       // min 0.5 turns total
```

### Kp Sweep

```cpp
float test_kp = (rpm_profile==0) ? imc_kp×0.75f : coggingSpeedP×0.75f;
// Ceiling: min(5× IMC baseline, 500k). Beyond this, encoder noise
// or cogging instantly saturates max_test_torque at any Kp.
float kp_tune_ceiling = min(imc_kp × 5.0f, 500000.0f);
test_dir = ±1 (alternates each test);

while (test_kp < kp_tune_ceiling && step_count < MAX_TUNE_STEPS(20)) {
    // 300ms coast settle before each sweep (always, including first)
    applySafeTorque(0);
    Delay(300);
    arm_pid_init_f32(&pid_soft, 1);

    // Trapezoidal velocity sweep:
    //   Phase 0: Accelerate at max_accel_turns_s2 to target_vel
    //   Phase 1: Cruise at target_vel → measure P2P error
    //   Phase 2: Decelerate at max_accel_turns_s2 to 0
    // Only the cruise phase error is evaluated.

    float p2p_deg = max_err_deg - min_err_deg;

    // --- CLAMP HANDLING ---
    // If |iq_cmd| hits 99% of max_test_torque → oscillation detected
    if (clamp_hit) {
        if (best_kp > 0) {
            test_kp /= 1.25f;
            if (test_kp <= best_kp × 1.01f) { tuning_done = true; break; }
        } else {
            test_kp /= 1.25f;
            if (test_kp < 50.0f) { tuning_done = true; break; }
        }
        sweep_up = false;
        test_dir = -test_dir;
        continue;
    }

    // --- CLAMP BACKOFF RECOVERY ---
    if (!sweep_up) {
        sweep_up = true;
        clamp_just_cleared = true; // use smaller step (×1.10) next
    }

    // --- P2P EVALUATION ---
    if (p2p_deg < lowest_p2p) { best_kp = test_kp; lowest_p2p = p2p_deg; }
    else if (p2p_deg > lowest_p2p × 1.5f && lowest_p2p < 1.0f) {
        if (!did_down_sweep) {
            test_kp = best_kp / 1.25f;
            did_down_sweep = true;
            sweep_up = false;
            test_dir = -test_dir;
            continue;
        } else {
            tuning_done = true;
            break;
        }
    }

    test_kp *= clamp_just_cleared ? 1.10f : 1.25f;
    clamp_just_cleared = false;
    test_dir = -test_dir;

    applySafeTorque(0);
    Delay(25);
}
```

### Store result

```cpp
this->coggingSpeedP = best_kp;
this->coggingSpeedI = 0.0f;
this->coggingSpeedD = 0.0f;
uint8_t store_src = (rpm_profile < COGGING_MAX_CALIB_PROFILES)
    ? rpm_profile : (COGGING_MAX_CALIB_PROFILES - 1);
this->cogging_calib_pidP[store_src] = (uint32_t)best_kp;
this->cogging_calib_pidI[store_src] = 0;
this->cogging_calib_pidD[store_src] = 0;
```

---

## STEP 3c — DFT Acquisition (CW + CCW sweeps)

Each profile runs **1 DFT iteration** with 2 directions.

### Per-direction sweep — Velocity Ramp + Coast Settle

```cpp
int8_t dirs[2] = {1, -1}; // CW then CCW

for (int8_t p : dirs) {
    float target_rpm = (p==1) ? calib_rpm : -calib_rpm;

    // --- COAST SETTLE ---
    // Zero torque for ~2 revolutions worth of time. RPM-proportional:
    // 5 RPM → 3s, 50 RPM → 2.4s, 256 RPM → 0.5s (min).
    applySafeTorque(0);
    float rev_s = 60.0f / fabsf(calib_rpm);
    uint32_t settle_ms = rev_s × 2000; // ~2 rev
    if (settle_ms < 500)  settle_ms = 500;
    if (settle_ms > 3000) settle_ms = 3000;
    // wait...
    arm_pid_init_f32(&pid_soft, 1); // fresh PID for sweep

    // --- VELOCITY RAMP ---
    // Ramp from 0 to full target velocity over cogging_warmup_ms.
    // Tiny floor (0.5% of target) ensures the target moves at least
    // ~1 encoder count/iteration on high-CPR encoders, preventing
    // stick-slip when the rotor is stuck in a cogging valley at t=0.
    float target_pos_f = getFilteredPosition();
    float full_vel_turns = target_rpm / 60.0f;
    float ramp_rate = full_vel_turns / cogging_warmup_ms × 1000;
    float ramp_vel_turns = 0.0f;

    while (HAL_GetTick() - calibStartTime < REVOLUTION_TIME_MS) {
        float elapsed = HAL_GetTick() - calibStartTime;
        if (elapsed < cogging_warmup_ms) {
            ramp_vel_turns = ramp_rate × elapsed × 0.001;
            if (ramp_vel_turns < fabsf(full_vel_turns) × 0.005f)
                ramp_vel_turns = fabsf(full_vel_turns) × 0.005f; // floor
        } else {
            ramp_vel_turns = full_vel_turns;
        }
        float step = ramp_vel_turns × dt_sec;
        target_pos_f += step;

        // PID + friction FF
        iq_pid = arm_pid_f32(&pid_soft, getWrappedError(target_pos_f, actual_pos_f));

        // Inertia FF: capped at 15% of max_test_torque to prevent encoder
        // quantization noise (1-count jitter on 2M CPR) from causing vibration
        iq_inertia = (J/100.0f) × current_accel_rad;
        if (cogging_calib_inertiaCorr)
            iq_pid += clip(iq_inertia, ±max_test_torque × 0.15);

        float iq_ff = sign(ramp_vel_turns) × dynamic_friction × 0.1f;
        iq_cmd = iq_pid + iq_ff;
        iq_applied = clip(iq_cmd + cog_comp, ±max_test_torque);

        if (fabs(iq_applied) >= max_test_torque × 0.99f) dft_clamped = true;
        applySafeTorque(iq_applied);
    }
    stopCalibTimers();
    applySafeTorque(0);
}
```

### DFT Clamp Retry — Unlimited

```cpp
// No retry limit — keeps dividing Kp by 1.25 until either
// DFT succeeds or Kp hits floor (50). Exits with warning if floor hit.
for (;;) {
    if (pid_soft.Kp < 50.0f || emergency) {
        broadcastCalibLog(0, "DFT clamp retries hit Kp floor (%.0f).", pid_soft.Kp);
        break;
    }
    // ... run DFT ...
    if (dft_clamped) {
        pid_soft.Kp /= 1.25f;
        if (pid_soft.Kp < 50.0f) pid_soft.Kp = 50.0f;
        continue;
    }
    break; // success
}
```

### DFT Demodulation

```cpp
// Runs only during valid data window (after warmup, within 1 revolution).
// Recursive harmonic accumulation using Chebyshev recurrence.

// DFT signal source (#ifdef COGGING_DFT_USE_IQ_CMD):
//   Defined:  float iq = iq_cmd;              // PID+friction only (phasor-add)
//   Not def:  float iq = actual_iq_raw;       // total applied torque
//             if (inertiaCorr) iq -= iq_inertia;

for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
    iq_acc_cos[k] += (iq × cur_c);
    iq_acc_sin[k] += (iq × cur_s);
    // Recurse: next_c = cur_c×c1 - cur_s×s1, next_s = cur_c×s1 + cur_s×c1
}
```

### Per-direction harmonic extraction

```cpp
float norm = 2.0f / dir_samples;
for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
    float re = iq_acc_cos[k] × norm, im = iq_acc_sin[k] × norm;
    if (p == 1) { cw_harms[k].mag = √(re²+im²); cw_harms[k].phase = atan2(re,im); }
    else        { ccw_harms[k].mag = √(re²+im²); ccw_harms[k].phase = atan2(re,im); }
}
```

### CW+CCW Combination (complex vector averaging)

```cpp
// Build top-20 combined harmonics:
for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
    float avg_mag = (cw_harms[k].mag + ccw_harms[k].mag) / 2.0f;

    // Convert phases to complex vectors → average → extract combined phase
    // (avoids wrap-around bugs near ±π)
    float cw_re = cosf(cw_harms[k].phase);
    float cw_im = sinf(cw_harms[k].phase);
    float ccw_re = cosf(ccw_harms[k].phase);
    float ccw_im = sinf(ccw_harms[k].phase);
    float avg_re = (cw_re + ccw_re) / 2.0f;
    float avg_im = (cw_im + ccw_im) / 2.0f;
    float avg_phase = atan2f(avg_im, avg_re);

    // Insertion sort into top-20 by magnitude
}

// Store CW/CCW raw harmonics for configurator offset tuning:
cw_store[0..19] = top CW harmonics;
ccw_store[0..19] = top CCW harmonics;
```

### DFT Clamp Retry (unlimited)

```cpp
// If acquisition saturated torque, Kp is too high for this RPM.
// Lower Kp and restart DFT. No retry limit — keeps dividing by
// 1.25 until either DFT succeeds or Kp hits floor (50).
for (;;) {
    if (pid_soft.Kp < 50.0f || emergency) {
        broadcastCalibLog(0, "DFT clamp retries hit Kp floor (%.0f).", pid_soft.Kp);
        break;
    }
    // ... run DFT ...
    if (dft_clamped) {
        pid_soft.Kp /= 1.25f;
        memset(active_tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
        applySafeTorque(0);
        Delay(250);
        continue;
    }
    break; // success
}
```

---

## STEP 3d — Phase-Lag Extraction

*Requires `COGGING_PHASE_SHIFT_MULTIRPM` to be defined (currently commented out).*

When active, tracks the dominant harmonic across all RPM profiles to extract
amplitude attenuation and phase lag vs the baseline profile:

```cpp
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
{
    static uint16_t master_dom_order = 1;
    static float master_ref_mag = 0.0f;

    // Lock dominant harmonic on profile 0 ONLY (never re-locked)
    if (rpm_profile == 0) {
        float max_cw_mag = 0.0f;
        for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
            if (cw_store[n].amplitude > max_cw_mag) {
                max_cw_mag = cw_store[n].amplitude;
                master_dom_order = cw_store[n].order;
            }
        }
    }

    // Get master harmonic's phase & magnitude from both directions
    float cw_ph=0, ccw_ph=0, cw_mg=0, ccw_mg=0;
    for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
        if (cw_store[n].order == master_dom_order)
            { cw_ph = cw_store[n].phase; cw_mg = cw_store[n].amplitude; }
        if (ccw_store[n].order == master_dom_order)
            { ccw_ph = ccw_store[n].phase; ccw_mg = ccw_store[n].amplitude; }
    }
    float avg_mag = (cw_mg + ccw_mg) / 2.0f;

    float phase_diff = cw_ph - ccw_ph;
    if (phase_diff > π) phase_diff -= 2π;
    if (phase_diff < -π) phase_diff += 2π;

    // Profile 0 = baseline (scale=1.0, phase=0.0°)
    if (rpm_profile == 0) {
        scale_curve_values[0] = 1.0f;
        phase_advance_curve_values[0] = 0.0f;
        master_ref_mag = avg_mag;
    }
    // Higher profiles: extract scale & phase lag
    else {
        float scale = avg_mag / master_ref_mag;
        scale = clip(scale, 0.1f, 3.0f);
        float lag_mech_rad = fabsf(phase_diff/2.0f) / (float)master_dom_order;
        float lag_mech_deg = lag_mech_rad × (180.0f/π);
        uint8_t idx = rpm_profile + 1;
        scale_curve_values[idx] = scale;
        phase_advance_curve_values[idx] = lag_mech_deg;
        scale_curve_count = max(scale_curve_count, idx+1);
    }
}
```

---

## STEP 4 — Save & Finalize

### Mark curves valid (guarded by `COGGING_DISABLE_SCALE_CURVE`)

```cpp
#ifndef COGGING_DISABLE_SCALE_CURVE
    scale_curve_valid = true;
    phase_adv_curve_valid = true;
#endif
```

### Fill all 24 breakpoints (nearest-neighbour interpolation)

```cpp
// When COGGING_DISABLE_SCALE_CURVE: use phase_adv values as calibration anchors
uint8_t calib_idx[24], calib_n = 0;
for (i = 0; i < 24; i++)
#ifdef COGGING_DISABLE_SCALE_CURVE
    if (phase_advance_curve_values[i] != 0.0f || i == 0) calib_idx[calib_n++] = i;
#else
    if (scale_curve_values[i] > 0.0f) calib_idx[calib_n++] = i;
#endif

// Interpolate between nearest calibrated neighbours
// Past last point: extrapolate using slope of last segment
#ifndef COGGING_DISABLE_SCALE_CURVE
    scale_curve_values[i] = interpolated;
#else
    scale_curve_values[i] = 1.0f; // force all to 1
#endif
    phase_advance_curve_values[i] = interpolated;

scale_curve_values[0] = 1.0f;
phase_advance_curve_values[0] = 0.0f;
```

### Precompute combined bins + Save to flash

```cpp
// Average CW and CCW bins into combined LUT (used by FF mode 1)
for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++)
    this->cogging_bins_combined[b] = (this->cw_bins[b] + this->ccw_bins[b]) * 0.5f;

saveCoggingTable();         // cogging_harmonics → flash
// saveFlash() only writes scale_curve if scale_curve_valid is true
```

---

## Verification Pass

After each profile's DFT completes, the firmware runs a **CW + CCW verification pass** (one revolution each direction) with the finalized feedforward active. The residual torque is collected into spatial bins (`ver_cw_bins[720]` / `ver_ccw_bins[720]`), and the top-20 residual harmonics are extracted and saved to `ver_cw_top[20]` / `ver_ccw_top[20]`.

```cpp
// Broadcast residual (order : amplitude : phase_rad):
VERIFY CW residual (order : amplitude : phase_rad):
1 : 115.7 : -0.0779
2 : 73.9 : 1.3866
...
```

Low residuals → good calibration. High residuals → try more DFT iterations or adjust PID.

The verification data is readable from the configurator via `coggingBins` command (adr 2-3 = ver bins, adr 4-5 = ver top-20 DFT).

---

### Combined DFT Harmonics vs Combined Bins

The system produces **two different "combined" datasets** from the CW and CCW sweeps:

| Dataset | How it's made | Used for |
|---|---|---|
| **Combined DFT harmonics** (`cogging_harmonics[]`) | Average CW & CCW DFT magnitudes; average phases via complex vector (cos+sin → average → atan2). Top 20 by magnitude. | Configurator graph, runtime FF mode 0 (harmonic sum) |
| **Combined bins** (`cogging_bins_combined[720]`) | Simple average of 720 CW and CCW spatial bins: `(cw_bins[b] + ccw_bins[b]) * 0.5f` | Runtime FF mode 1 (bin LUT) |

The configurator's **Harmonic Editor** graph always shows the **combined DFT harmonics** — it reconstructs a smooth sine wave by summing `amp × sin(order × θ + phase)` for each harmonic. This is why the graph appears centered on the DFT reconstruction (smooth, sinusoidal) rather than the raw bins (jagged, noisy, with friction asymmetry preserved).

---

## STEP 5 — Return to Center

```cpp
// Compute distance and return RPM
actual_pos_f = getAbsolutePosition();
distance_turns = fabs(actual_pos_f);
ret_rpm = distance_turns × 6.0; // 10-second return
if (ret_rpm < 0.5) ret_rpm = 0.5;

// Kp: find nearest breakpoint ≤ ret_rpm, use its stored auto-tuned Kp.
// This ensures Kp is safe for the actual return speed (not calib_rpm).
// E.g., 10 turns away → 60 RPM return → Kp from 50 RPM profile.

// Clamp retry (up to 3 tries):
//   If |iq_applied| ≥ max_test_torque × 0.99 during return:
//     Kp = Kp / 1.25, delay 250ms, restart from current position.
//   If retries exhausted, proceed with whatever Kp we have.

timeout = 15000; // 10s travel + 5s safety margin
// Velocity ramp over cogging_warmup_ms, anti-cogging active
// Stop at position ≈ 0 ± 0.005 turns
```

---

## Runtime: `turn()` Compensation

```cpp
void TMC4671::turn(int16_t power) {
    if (cogging_enabled && !isCalibrationInProgress()) {
        float pos_f = getFilteredPosition();

        // Measure RPM from position delta
        float signed_rpm = (pos_f - prev_pos) / dt × 60;
        measured_rpm = fabsf(signed_rpm);

        // --- DO NOT SHIFT pos_f ---
        // Motor inductance acts as an RL low-pass filter — phase delay
        // physically cannot exceed 90° electrical. Shifting pos_f mechanically
        // multiplies the advance by harmonic order, over-shifting high harmonics
        // past 90° (sometimes 180°+) which creates texture/bumps at high RPM.
        // Instead we compute the mechanical advance separately and apply it
        // ONLY to harmonics at or below the dominant order.
        float adv_mech_rad = 0.0f;
        if (phase_adv_curve_valid) {
            float adv_deg = interpolatePhaseAdvance(measured_rpm);
            float dir = (signed_rpm >= 0) ? 1 : -1;
            adv_mech_rad = dir × (adv_deg / 360.0f) × 2π;
        }

        // Per-RPM harmonic blending or base table
#ifdef COGGING_DISABLE_BLEND
        Harmonic* blended = cogging_harmonics;
#else
        Harmonic blended[COGGING_HARMONICS_COUNT];
        blendHarmonicTables(measured_rpm, blended);
#endif

        float angle_rad = pos_f × 2π;

        // 1. Find dominant harmonic
        float dom_amp = 0, dom_phase = 0;
        uint16_t dom_order = 1;
        for (i = 0; i < COGGING_HARMONICS_COUNT; i++) {
            if (blended[i].amplitude > dom_amp) {
                dom_amp = blended[i].amplitude;
                dom_order = (uint16_t)blended[i].order;
                dom_phase = blended[i].phase;
            }
        }

        // 2. Fourier sum with per-harmonic electrical advance
        //    Only harmonics ≤ dom_order get the advance — higher harmonics
        //    stay anchored to their physical spatial positions.
        float compensation = 0;
        for (i = 0; i < COGGING_HARMONICS_COUNT; i++) {
            if (blended[i].amplitude > 0) {
                float elec_adv = 0.0f;
                if (blended[i].order <= dom_order)
                    elec_adv = blended[i].order × adv_mech_rad;
                compensation += blended[i].amplitude
                    × sin(angle_rad × blended[i].order + blended[i].phase + elec_adv);
            }
        }

        // 3. H3 waveshaping (tracks dominant wave with its own advance)
        if (h3_shaping != 0.0f && dom_amp > 0.0f) {
            float dom_elec_adv = dom_order × adv_mech_rad;
            float shaped_arg = h3_mult × (dom_order × angle_rad + dom_phase + dom_elec_adv)
                             + h3_phase_trim;
            compensation -= h3_shaping × dom_amp × sin(shaped_arg);
        }

        // Speed-dependent scale (or constant cogging_scale)
        float dyn_scale = scale_curve_valid
            ? interpolateScale(measured_rpm) : cogging_scale;

        last_anticogging_torque = dyn_scale × compensation;
        totalPower += last_anticogging_torque;
    }
    setFluxTorque(flux, totalPower);
}
```

### Why per-harmonic advance instead of shifting `pos_f`?

| Method | Effect on high harmonics |
|---|---|
| Shift `pos_f` (old) | 5° mechanical → 50° on order-10, 100° on order-20 — exceeds 90° RL limit, flips phase |
| Per-harmonic advance (new) | Advance limited to harmonics ≤ dominant order; high-frequency texture stays at physical positions |

The RL low-pass characteristic of motor coils physically caps electrical phase lag at 90°. Multiplying a mechanical shift by harmonic order artificially pushes high harmonics past this limit, destroying the spatial texture. By applying the advance only to the dominant (and lower) harmonics, the main cogging wave is compensated without over-shifting the texture.

---

## Flash Load (`loadFlash`) — Power-Cycle Recovery

```cpp
// scale_curve_valid guarded by COGGING_DISABLE_SCALE_CURVE:
#ifndef COGGING_DISABLE_SCALE_CURVE
    if (any_valid && scale_curve_values[1] >= 1.0f)
        scale_curve_valid = true;
#endif

// phase_adv_curve_valid restored from flash (not guarded):
if (padv_any_valid)
    phase_adv_curve_valid = true;

// Fill all 24 breakpoints (same interpolation as STEP 4)
```

**Note**: When `COGGING_DISABLE_SCALE_CURVE` is defined, calibration does not set `phase_adv_curve_valid`. But `loadFlash()` can restore it from old flash data. Erase the flash sector for a clean disable.

---

## Blend Tables (when `COGGING_DISABLE_BLEND` is NOT defined)

| Table | Profiles | Description |
|---|---|---|
| `cogging_harmonics` | Profile 0 | Low-RPM base table |
| `cogging_harmonics_rpm2` | Profile 1 | Mid-RPM table |
| `cogging_harmonics_rpm3` | Profile 2+ | High-RPM table |

Blending at runtime:
- `rpm < blend_rpm1`: 100% base
- `blend_rpm1 ≤ rpm < blend_rpm2`: crossfade base → rpm2
- `rpm ≥ blend_rpm2`: crossfade rpm2 → rpm3

When `COGGING_DISABLE_BLEND`: only base table used at all RPMs.

---

## Key Variables Reference

| Name | Type | Description |
|---|---|---|
| `cogging_harmonics[20]` | `Harmonic[]` | Spatial anti-cogging Fourier table |
| `cogging_harmonics_rpm2[20]` | `Harmonic[]` | Mid-RPM spatial table |
| `cogging_harmonics_rpm3[20]` | `Harmonic[]` | High-RPM spatial table |
| `scale_curve_values[24]` | `float[]` | Attenuation per RPM |
| `phase_advance_curve_values[24]` | `float[]` | Phase advance (mech deg) per RPM |
| `scale_curve_rpm_points[24]` | `float[]` | Fixed RPM breakpoints `{0,5,7,10,…,256}` |
| `scale_curve_count` | `uint8_t` | Calibrated breakpoint count (max 24) |
| `scale_curve_valid` | `bool` | Scale curve calibrated & not disabled |
| `phase_adv_curve_valid` | `bool` | Phase curve calibrated & not disabled |
| `rpm2_table_valid` | `bool` | Mid-RPM table available |
| `rpm3_table_valid` | `bool` | High-RPM table available |
| `blend_rpm1/2/3` | `float` | Blend anchor RPMs |
| `master_dom_order` | `static uint16_t` | Dominant harmonic (locked at profile 0) |
| `master_ref_mag` | `static float` | Reference magnitude at profile 0 |
| `cw_store[20]` | `Harmonic[]` | Top-20 CW DFT harmonics |
| `ccw_store[20]` | `Harmonic[]` | Top-20 CCW DFT harmonics |
| `cogging_calib_pidP/I/D[5]` | `uint32_t[]` | Stored gains per profile slot |
| `coggingSpeedP/I/D` | `float` | Current active PID gains |
| `J` | `float` | Motor inertia (TMC units = physical×100) |
| `B` | `float` | Viscous friction |
| `cogging_warmup_ms` | `uint32_t` | Ramp-up duration (J-scaled, capped 8000ms) |
| `cogging_calib_autoPid` | `bool` | Enable per-profile P-gain auto-tuning |
| `cogging_calib_inertiaCorr` | `bool` | Enable inertia FF during DFT |
| `h3_shaping` | `float` | 3rd-harmonic waveshaping amount |
| `cogging_scale` | `float` | Global anti-cogging scale (1.0 = full) |
| `max_test_torque` | `float` | Torque limit = `bangInitPower × 0.8` |

---

## Change Log

| Date | Change |
|---|---|
| 2026-07-02 | DFT: RPM-proportional coast settle (2 rev worth, 0.5–3s) replaces fixed warmup×2 |
| 2026-07-02 | DFT: minimum ramp velocity floor (0.5% of target) prevents stick-slip on high-CPR encoders |
| 2026-07-02 | DFT: unlimited clamp retry — keeps dividing Kp until floor at 50 |
| 2026-07-02 | DFT: removed `init_kick` — instant position offset × high Kp = instant clamp |
| 2026-07-02 | `turn()`: per-harmonic electrical advance instead of shifting `pos_f` |
| 2026-07-02 | P-tuner: ceiling capped at `min(imc_kp × 5, 500000)` |
| 2026-07-02 | P-tuner: coast settle before every sweep (not just between direction changes) |
| 2026-07-02 | IMC validation: no longer aborts on incomplete quarter coverage |
| 2026-07-02 | Return-to-center: Kp matched to actual return RPM (≤ nearest breakpoint), clamp retry |
| 2026-07-02 | Inertia correction: cap reduced to 15% of max_test_torque (encoder noise protection) |
| 2026-07-01 | DFT: velocity ramp from 0 during warmup |
| 2026-07-01 | DFT: friction FF uses `ramp_vel_turns` |
| 2026-07-01 | Return-to-center: fixed 10-second duration (RPM = distance × 6) |
| 2026-07-01 | `COGGING_DISABLE_SCALE_CURVE`: guards in `loadFlash()`, MULTIRPM, finalize |
| 2026-07-01 | `COGGING_DISABLE_BLEND`: fixed `blended` pointer bug |
| 2026-07-01 | Array bounds: PID storage wrapped to `COGGING_MAX_CALIB_PROFILES-1` |
