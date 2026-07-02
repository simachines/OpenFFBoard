# OpenFFBoard Anti-Cogging Calibration — Step-by-Step Guide

> **Active defines:** `COGGING_TABLE_FLASH_START_ADDRESS`, `COGGING_PHASE_SHIFT_MULTIRPM`
>
> **Optional defines:** `COGGING_DISABLE_SCALE_CURVE`, `COGGING_DISABLE_BLEND`, `COGGING_DFT_USE_IQ_CMD`

---

## Overview

The calibration measures motor cogging torque and computes speed-dependent phase lag.  
It runs **23 RPM profiles** (all `scale_curve_rpm_points` except RPM 0 plateau), each with:

1. **P-gain auto-tuning** — finds optimal velocity-loop Kp via trapezoidal sweeps
2. **DFT acquisition** — CW/CCW constant-speed revolutions while recording IQ torque
3. **Phase-lag extraction** — computes attenuation & phase shift from CW/CCW demodulation

Total output:

- `cogging_harmonics[20]` — spatial anti-cogging Fourier table (saved on profile 0 only)
- `scale_curve_values[24]` — attenuation vs RPM (unless `COGGING_DISABLE_SCALE_CURVE`)
- `phase_advance_curve_values[24]` — phase shift in mechanical degrees vs RPM (unless `COGGING_DISABLE_SCALE_CURVE`)
- `scale_curve_rpm_points[24]` — RPM breakpoints (fixed, configurator-compatible)

---

## Compile-Time Feature Switches

| Define | Effect when defined | Effect when NOT defined |
|---|---|---|
| `COGGING_PHASE_SHIFT_MULTIRPM` | Tests all 23 fixed RPM breakpoints; extracts phase lag per RPM via master harmonic tracking | User-configured RPM profiles |
| `COGGING_DISABLE_SCALE_CURVE` | **No scale curve** — `cogging_scale` stays at 1.0 at all RPMs; `phase_adv_curve_valid` also stays false | Scale & phase curves are populated and applied at runtime |
| `COGGING_DISABLE_BLEND` | Uses only `cogging_harmonics` base table at all RPMs | Blends 3 per-RPM harmonic tables based on measured speed |
| `COGGING_DFT_USE_IQ_CMD` | DFT uses `iq_cmd` (PID+friction only) — phasor-add mode for iterative refinement | DFT uses `actual_iq_raw` (total torque) — direct replacement on each iteration |

### `COGGING_DISABLE_SCALE_CURVE` details

When defined, the following are ALL disabled:

- `scale_curve_valid` — never set to true (calibration or loadFlash)
- `phase_adv_curve_valid` — never set to true (calibration only; loadFlash still restores it)
- `scale_curve_values[]` writes in MULTIRPM block — skipped
- Load-time `scale_curve_valid` restoration — skipped
- Fill-in interpolation for scale values — skipped

Runtime effect: `turn()` uses `cogging_scale` (1.0) directly, no phase-advance shift.

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
| `multirpm_count` | `uint8_t` | `23` | Number of profiles = `SCALE_CURVE_POINTS - 1` |
| `master_dom_order` | `static uint16_t` | `1` then locked | Dominant harmonic order from profile 0 |
| `master_ref_mag` | `static float` | `0.0f` then set | Reference magnitude at profile 0 |

### Curve arrays zeroed

```cpp
memset(scale_curve_values, 0, sizeof(scale_curve_values));
memset(phase_advance_curve_values, 0, sizeof(phase_advance_curve_values));
// Plateau always scale=1.0, phase=0.0°
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
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
    uint8_t multirpm_count = SCALE_CURVE_POINTS - 1; // 23 profiles
    for (uint8_t rpm_profile = 0; rpm_profile < multirpm_count; rpm_profile++) {
        calib_rpm = scale_curve_rpm_points[rpm_profile + 1]; // skip RPM 0
        // calib_rpm = 5, 7, 10, 12, 15, 20, 25, ..., 256
#else
    for (rpm_profile = 0; rpm_profile <= cogging_calib_count; rpm_profile++) {
        calib_rpm = cogging_calib_rpm[rpm_profile]; // user-configured
#endif
```

### 3a — Per-Profile PID Loading

```cpp
// MULTIRPM: wrap profile index to COGGING_MAX_CALIB_PROFILES-1 for array safety
uint8_t pid_src = rpm_profile;
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
    if (pid_src >= COGGING_MAX_CALIB_PROFILES) pid_src = COGGING_MAX_CALIB_PROFILES - 1;
#endif
pid_soft.Kp = cogging_calib_pidP[pid_src];
pid_soft.Ki = cogging_calib_pidI[pid_src];
pid_soft.Kd = cogging_calib_pidD[pid_src];

// If all zeros (unconfigured slot), copy from slot 0 (IMC baseline):
if (pid_src > 0 && pidP == 0 && pidI == 0 && pidD == 0) {
    pidP = cogging_calib_pidP[0]; // fallback to IMC
}
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
test_dir = ±1 (alternates each test);

while (test_kp < KP_TUNE_CEILING && step_count < MAX_TUNE_STEPS(20)) {
    // 300ms settle at zero torque between direction changes
    if (step_count > 1) { applySafeTorque(0); Delay(300); }

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
coggingSpeedP = best_kp;
coggingSpeedI = 0.0f;
coggingSpeedD = 0.0f;
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
    uint8_t store_src = min(rpm_profile, COGGING_MAX_CALIB_PROFILES-1);
    cogging_calib_pidP[store_src] = (uint32_t)best_kp;
    cogging_calib_pidI[store_src] = 0;
    cogging_calib_pidD[store_src] = 0;
#else
    cogging_calib_pidP[rpm_profile] = (uint32_t)best_kp;
#endif
```

---

## STEP 3c — DFT Acquisition (CW + CCW sweeps)

Each profile runs **1 DFT iteration** with 2 directions.

### Per-direction sweep — Velocity Ramp + Active Settle

```cpp
int8_t dirs[2] = {1, -1}; // CW then CCW

for (int8_t p : dirs) {
    float target_rpm = (p==1) ? calib_rpm : -calib_rpm;

    // --- ACTIVE SETTLE ---
    // Hold position with 30% torque limit for 1.5s to BRING MOTOR TO STOP.
    // Passive coasting isn't enough at higher RPMs.
    {
        float hold_pos = getFilteredPosition();
        uint32_t settleStart = HAL_GetTick();
        while (HAL_GetTick() - settleStart < 1500 && ...) {
            float err = getWrappedError(hold_pos, getFilteredPosition());
            float iq_hold = clip(arm_pid_f32(&pid_soft, err),
                                 -max_test_torque×0.3, max_test_torque×0.3);
            applySafeTorque(iq_hold);
            Delay(1);
        }
        applySafeTorque(0);
        Delay(50); // let PID state bleed off
    }

    // --- VELOCITY RAMP ---
    // Ramp from 0 to full target velocity over cogging_warmup_ms.
    float target_pos_f = getFilteredPosition();
    float prev_vel_turns = 0.0f;           // start from rest
    float full_vel_turns = target_rpm / 60.0f;
    float ramp_vel_turns = 0.0f;
    float ramp_rate = full_vel_turns / cogging_warmup_ms × 1000; // turns/s²

    uint32_t rev_ms = (60.0f/calib_rpm) × 1500;
    uint32_t REVOLUTION_TIME_MS = rev_ms + cogging_warmup_ms;

    startCalibTimers(TIM_TMC_ARR);
    while (HAL_GetTick() - calibStartTime < REVOLUTION_TIME_MS) {
        float elapsed = HAL_GetTick() - calibStartTime;
        if (elapsed < cogging_warmup_ms)
            ramp_vel_turns = ramp_rate × elapsed × 0.001;
        else
            ramp_vel_turns = full_vel_turns;

        float step = ramp_vel_turns × dt_sec;
        target_pos_f += step;

        float error = getWrappedError(target_pos_f, actual_pos_f);
        iq_pid = arm_pid_f32(&pid_soft, error);

        // Inertia FF (if cogging_calib_inertiaCorr enabled)
        iq_inertia = (J/100.0f) × current_accel_rad;
        if (cogging_calib_inertiaCorr) iq_pid += iq_inertia;

        // Friction FF: 10% of dynamic_friction, scaled by ramp_vel_turns
        float iq_ff = sign(ramp_vel_turns) × dynamic_friction × 0.1f;

        iq_cmd = iq_pid + iq_ff;

        // Cogging feed-forward from active_tbl (empty on iter 0)
        float cog_comp = Σ active_tbl[h].amplitude × sin(angle×order + phase);
        float iq_applied = iq_cmd + cogging_scale × cog_comp;

        if (fabs(iq_applied) >= max_test_torque × 0.99f) dft_clamped = true;

        applySafeTorque(iq_applied);
    }
    stopCalibTimers();
    applySafeTorque(0);
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

### CW+CCW Combination (Piccoli phase alignment)

```cpp
// Build top-20 combined harmonics:
for (int k = 5; k < COGGING_CALIB_DFT_HARMONICS; k++) {
    float avg_mag = (cw_harms[k].mag + ccw_harms[k].mag) / 2.0f;

    // Phase unwrapping:
    float phase_diff = cw_phase - ccw_phase;
    if (phase_diff > π) ccw_phase += 2π;
    if (phase_diff < -π) ccw_phase -= 2π;
    float avg_phase = (cw_phase + ccw_phase) / 2.0f;

    // Insertion sort into top-20 by magnitude
}

// Broadcast CW/CCW raw data for configurator:
cw_store[0..19] = top CW harmonics;
ccw_store[0..19] = top CCW harmonics;
```

### DFT Clamp Retry

```cpp
// If acquisition saturated torque, Kp is too high for this RPM.
// Lower Kp and restart DFT (up to DFT_MAX_CLAMP_RETRIES(3) times).
if (dft_clamped && retries_remaining) {
    pid_soft.Kp /= 1.25f;
    coggingSpeedP = pid_soft.Kp;
    memset(active_tbl, 0, ...);
    applySafeTorque(0);
    Delay(250);
    continue;
}
```

---

## STEP 3d — Phase-Lag Extraction (MULTIRPM)

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
```

### Profile 0 (baseline)

```cpp
    if (rpm_profile == 0) {
#ifndef COGGING_DISABLE_SCALE_CURVE
        scale_curve_values[0] = 1.0f;
#endif
        phase_advance_curve_values[0] = 0.0f;
        master_ref_mag = avg_mag;
    }
```

### Higher profiles (extract phase lag & attenuation)

```cpp
    else {
        float scale = avg_mag / master_ref_mag;
        scale = clip(scale, 0.1f, 3.0f);

        float lag_mech_rad = fabsf(phase_diff/2.0f) / (float)master_dom_order;
        float lag_mech_deg = lag_mech_rad × (180.0f/π);

        uint8_t idx = rpm_profile + 1;
#ifndef COGGING_DISABLE_SCALE_CURVE
        scale_curve_values[idx] = scale;
#endif
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

### Save to flash

```cpp
saveCoggingTable();         // cogging_harmonics → flash
// saveFlash() only writes scale_curve if scale_curve_valid is true
```

---

## STEP 5 — Return to Center

```cpp
// Fixed 10-second return regardless of distance
actual_pos_f = getAbsolutePosition();  // may be many turns from center
distance_turns = fabs(actual_pos_f);

// RPM = distance / (10s / 60s) = distance × 6
ret_rpm = distance_turns × 6.0;
if (ret_rpm < 0.5) ret_rpm = 0.5;      // minimum speed
direction = (actual_pos_f > 0) ? -ret_rpm : +ret_rpm;

timeout = 15000; // 10s travel + 5s safety margin

// Uses Kp auto-tuned for nearest calibrated RPM
// Velocity ramp from 0 over cogging_warmup_ms
// Anti-cogging compensation active during return
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

        // Phase-advance position shift (if phase_adv_curve_valid)
        if (phase_adv_curve_valid) {
            float adv_deg = interpolatePhaseAdvance(measured_rpm);
            float dir = (signed_rpm >= 0) ? 1 : -1;
            pos_f += dir × adv_deg / 360.0f;
            pos_f = pos_f - floorf(pos_f);
        }

        // Per-RPM harmonic blending or base table
#ifdef COGGING_DISABLE_BLEND
        Harmonic* blended = cogging_harmonics; // base table only
#else
        Harmonic blended[COGGING_HARMONICS_COUNT];
        blendHarmonicTables(measured_rpm, blended);
#endif

        // Fourier sum
        float compensation = 0;
        for (i = 0; i < COGGING_HARMONICS_COUNT; i++) {
            if (blended[i].amplitude > 0)
                compensation += amplitude × sin(2π×pos_f×order + phase);
        }

        // H3 waveshaping (if enabled)
        if (h3_shaping != 0.0f && dom_amp > 0.0f) {
            compensation -= h3_shaping × dom_amp × sin(h3_mult×(dom_order×angle+dom_phase) + h3_phase_trim);
        }

        // Speed-dependent scale
        float dyn_scale = scale_curve_valid ? interpolateScale(measured_rpm) : cogging_scale;

        last_anticogging_torque = dyn_scale × compensation;
        totalPower += last_anticogging_torque;
    }
    setFluxTorque(flux, totalPower);
}
```

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
| 2026-07-01 | DFT: velocity ramp from 0 during warmup (prevents direction-change torque spike) |
| 2026-07-01 | DFT: active position-hold settle (1.5s, 30% torque) replaces passive 1s wait between CW/CCW |
| 2026-07-01 | DFT: friction FF uses `ramp_vel_turns` (matches actual velocity during ramp) |
| 2026-07-01 | Return-to-center: fixed 10-second duration (RPM = distance × 6) |
| 2026-07-01 | P-tuner: time-capped sweeps (`min(total_dist, target_vel×3.0)`) for fast 3 RPM tuning |
| 2026-07-01 | `COGGING_DISABLE_SCALE_CURVE`: guards `scale_curve_valid` in `loadFlash()`, MULTIRPM writes, and `phase_adv_curve_valid` in finalize |
| 2026-07-01 | `COGGING_DISABLE_BLEND`: fixed `blended` pointer bug (reading uninitialized stack) |
| 2026-07-01 | Array bounds: PID storage wrapped to `COGGING_MAX_CALIB_PROFILES-1` in MULTIRPM |
| 2026-07-01 | Return-to-center Kp: looks up nearest RPM's stored auto-tuned Kp |
