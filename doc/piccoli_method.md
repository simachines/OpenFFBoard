# The Piccoli Method — Anti-Cogging Theory & Practice

> Based on: *"Anticogging: Torque Ripple Suppression, Modeling, and Parameter Selection"*
> by Matthew Piccoli and Mark Yim, University of Pennsylvania.
> *The International Journal of Robotics Research*, Vol. 35(1-3):148–160, 2016.
>
> Full paper: [https://www.modlabupenn.org/wp-content/uploads/piccoli_matthew_anticogging_torque_ripple_suppression_modeling_and_parameter_selection.pdf](https://www.modlabupenn.org/wp-content/uploads/piccoli_matthew_anticogging_torque_ripple_suppression_modeling_and_parameter_selection.pdf)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Types of Torque Ripple](#2-types-of-torque-ripple)
3. [Core Principle: CW/CCW Averaging](#3-core-principle-cwccw-averaging)
4. [Waveform Collection Methods](#4-waveform-collection-methods)
5. [Waveform Analysis](#5-waveform-analysis)
6. [Waveform Suppression (Playback)](#6-waveform-suppression-playback)
7. [Torque Ripple Modeling](#7-torque-ripple-modeling)
8. [Practical Guidelines for Parameter Selection](#8-practical-guidelines-for-parameter-selection)
9. [Relation to OpenFFBoard Implementation](#9-relation-to-openffboard-implementation)
10. [References](#10-references)

---

## 1. Introduction

The Piccoli method is a low-cost technique for measuring and canceling cogging torque (detent torque) in
permanent-magnet synchronous motors (PMSMs) using only a position sensor — which is already present
for servo control. The core insight is that **cogging torque manifests in the mechanical state** (position
and velocity) and can therefore be observed without expensive force/torque sensors.

**Key results from the paper:**
- Up to 88% reduction in RMS torque ripple on hobby-grade brushless motors
- Comparable or better performance than motors costing over 9× more
- Two collection methods: **position-based** (hold & measure) and **acceleration-based** (spin & differentiate)
- A complete model of torque ripple sources to guide PWM frequency and resolution selection

---

## 2. Types of Torque Ripple

The paper identifies four sources of torque ripple in PMSMs:

| Source | Current-dependent? | Frequency | Direction-symmetric? |
| :--- | :--- | :--- | :--- |
| **Mutual torque** | Yes | Electrical ($p \times$ mechanical) | Yes |
| **Reluctance torque** | Yes | $2\times$ electrical | Yes |
| **Cogging torque** (detent) | **No** | Electrical ($p \times$ mechanical) | **Yes** |
| **Friction torque** | No | 1× mechanical | **No** (sign changes with direction) |

**Cogging torque** is caused by the rotor's permanent magnets attracting the salient stator teeth.
It is not detectable via current sensors — only mechanical sensors (position, velocity, acceleration)
can see it. It has no DC component (zero mean over one revolution).

**Friction torque** is distinguished from cogging by:
- Once-per-revolution frequency (vs. electrical frequency for cogging)
- **Sign change upon direction reversal** — friction opposes motion, so forward and backward
  friction torques have opposite signs relative to the cogging torque

This last property is the foundation of the CW/CCW averaging method.

---

## 3. Core Principle: CW/CCW Averaging

The key mathematical insight of the Piccoli method is that **cogging torque is direction-symmetric
while friction torque is direction-antisymmetric**.

If you measure the total torque ripple in the forward (CW) direction and the backward (CCW) direction:

$$\tau_{fw}(\theta) = \tau_{cog}(\theta) + \tau_{fr}(\theta)$$
$$\tau_{bw}(\theta) = \tau_{cog}(\theta) - \tau_{fr}(\theta)$$

Then:

$$\tau_{cog}(\theta) = \frac{\tau_{fw}(\theta) + \tau_{bw}(\theta)}{2}$$

$$|\tau_{fr}(\theta)| = \frac{|\tau_{fw}(\theta) - \tau_{bw}(\theta)|}{2}$$

This averaging cancels friction bias and isolates the pure cogging torque. This is the
foundation used by the OpenFFBoard's anti-cogging implementation.

### Phase alignment

Since CW and CCW sweeps also have opposite PID tracking lag (the controller lags slightly behind
the target in both directions), the phases of individual harmonics shift in opposite directions.
Averaging the phases cancels this tracking-lag error.

The paper's implementation uses FFT fitting to extract harmonic coefficients. The OpenFFBoard
uses a continuous DFT-128 with Chebyshev recurrence instead, but the averaging principle is identical.

---

## 4. Waveform Collection Methods

The paper presents two methods for collecting torque ripple data. Both require only a position sensor.

### 4.1 Position-Based Method (Algorithm 1)

A high-gain proportional position controller commands the rotor to a sequence of positions
$\theta_{m,cmd,i}$ in monotonically increasing order. At each position, the controller waits for
the rotor to come to a complete stop ($\dot{\theta}_m = 0$) so that $dI/dt = 0$, allowing
simplification of the motor model to $V_{app} = IR$.

Recorded at each position:
- Actual position $\theta_{m,act,i}$
- Applied PWM duty cycle $d_i$ (in per-unit, $d \in [0, 1]$)
- Supply voltage $V_{sup,i}$
- Current $I_i$

The process is repeated with positions in decreasing order to get the reverse-direction waveform.

**Advantage:** Works for constrained joints (sub-rotation intervals).  
**Disadvantage:** Slower; requires stopping at each position.

### 4.2 Acceleration-Based Method (Algorithm 2)

**This is the method the paper calls "acceleration-based" — and it is fundamentally different
from the OpenFFBoard's approach (see [section 9](#9-relation-to-openffboard-implementation)).**

The motor is spun at the minimum possible **open-loop** speed — no velocity feedback, no PID.
A constant PWM duty cycle $d_{min}$ is applied and the rotor accelerates/decelerates purely
under the influence of cogging torque. The process:

1. Increment duty cycle until the motor overcomes the largest cog and starts moving — this
   gives $d_{max}$, the duty cycle that overcomes maximum cogging + stiction + deadtime.
2. Decrement duty cycle until the motor stops — this gives $d_{min}$, the minimum duty
   cycle for steady rotation.
3. Restart with $d_{max}$, then switch to $d_{min}$ once steady-state is reached.
4. Record position $\theta_{m,j}$, **velocity** $\dot{\theta}_{m,j}$, and time $t_j$ over $n$ revolutions.
   (Velocity is sampled by counting encoder counts in a fixed time period, or timing a fixed
   number of encoder counts.)

Repeat in the opposite direction.

**The name "acceleration-based":** With a constant open-loop duty cycle, variations in rotor
speed are caused by cogging torque alternately aiding and opposing motion. The recorded velocity
is numerically differentiated to get acceleration:

$$\ddot{\theta}_{m,i} = \frac{d}{dt} \text{FFT}(\dot{\theta}_{m,j})$$

Then, knowing rotor inertia $J$, cogging torque is:

$$\tau_{cog,i} = J \ddot{\theta}_{m,i}$$

The acceleration IS the cogging signal — this is the sense in which the method is
"acceleration-based."

**Key characteristics:**
- **Open-loop** (no velocity feedback)
- Measures velocity ripple, differentiates to get acceleration
- Requires known or estimated rotor inertia $J$ to convert to torque
- Faster than the position method; collects continuous data

---

## 5. Waveform Analysis

### 5.1 Position-Based Analysis (Algorithm 3)

Raw data from Algorithm 1 contains multiple readings per position (from forward and backward
passes). The analysis:

1. **Consolidate duplicates**: For each encoder position $i$, store the max and min of:
   - Commanded duty cycles: $d_{max,i}$, $d_{min,i}$
   - Currents: $I_{max,i}$, $I_{min,i}$

2. **Extract cogging waveform**:
   $$d_{cog,i} = \frac{d_{max,i} + d_{min,i}}{2}$$
   $$I_{cog,i} = \frac{I_{max,i} + I_{min,i}}{2}$$

3. **Extract deadtime+stiction band**:
   $$d_{dt,st,i} = \frac{d_{max,i} - d_{min,i}}{2}$$

   The maximum of this across all positions gives $d_{dt,stmax}$. Values below this threshold
   are averaged to get $d_{dt,st}$; values above give the stiction-only component $d_{st}$.

4. **Separate deadtime from stiction**:
   $$d_{dt} = \overline{d_{dt,st}} - \overline{d_{st}}$$

### 5.2 Acceleration-Based Analysis (Algorithm 4)

Since the motor runs open-loop with constant duty cycle, velocity varies with position.
Velocity data is FFT-fitted to fill gaps and make it differentiable:

$$\ddot{\theta}_{m,i} = \frac{d}{dt} \text{FFT}(\dot{\theta}_{m,j})$$

Cogging torque follows from Newton's second law ($\tau = J\ddot{\theta}$), and the cogging
duty cycle is scaled by the $d_{min}$ reference:

$$d_{cog,i} = d_{min} \frac{\ddot{\theta}_{m,i}}{\max(\ddot{\theta}_{m,i})}$$

**Important:** This method requires knowing $J$ (rotor inertia) or assumes it's constant.
The OpenFFBoard's approach does not need $J$ because it measures torque output directly
from the PID controller.

### 5.3 FFT Fitting

Since torque ripple is periodic with mechanical angle, a Fourier series is a natural fit:

$$d_{cog}(\theta) = \sum_{k=1}^{N} A_k \sin(k \theta + \phi_k)$$

The paper uses FFTs (or bi-cubic splines) to fill data gaps. The OpenFFBoard performs
continuous Goertzel/Chebyshev DFT accumulation during the sweep, extracting the top-20
harmonics by magnitude — conceptually equivalent but done in a single pass.

---

## 6. Waveform Suppression (Playback)

Once the cogging map is known, it is injected as a feedforward term:

**Voltage control:**
$$V_{out} = V_{des} + \text{sgn}(V_{des}) V_{st,i} + V_{cog,i}$$
$$d = \frac{V_{out}}{V_{sup}} + \text{sgn}(V_{out}) d_{dt}$$

**Current control:**
$$I = I_{des} + \text{sgn}(I_{des}) I_{st,i} + I_{cog,i}$$

The feedforward can be implemented as:
- **FFT harmonic sum** — evaluate the Fourier series at runtime (low order only)
- **Lookup table** — precomputed $V_{cog,i}$ or $I_{cog,i}$ indexed by encoder position

The OpenFFBoard supports both: FF mode 0 uses the harmonic sum (Fourier series from
`cogging_harmonics[]`), while modes 1 and 2 use the bin LUT (spatial lookup table
from `cogging_bins_combined[]` or per-direction bins).

---

## 7. Torque Ripple Modeling

The paper develops a model of all torque ripple sources to guide design parameter selection,
primarily PWM frequency and resolution. All sources are assumed uncorrelated, so total
RMS torque ripple is:

$$\tau_{RMS} = \sqrt{\tau_{res}^2 + \tau_{frq}^2 + \tau_{dt}^2 + \tau_{enc}^2 + \tau_{cog}^2 + \tau_{fr}^2 + \tau_{mtl}^2}$$

### Source equations

| Source | Symbol | Equation | Key dependency |
| :--- | :--- | :--- | :--- |
| PWM resolution | $\tau_{res}$ | $\frac{V_{sup} f_{pwm} K_\tau}{R f_{clk} \sqrt{3}}$ | $\propto f_{pwm}$ (higher = worse) |
| PWM frequency | $\tau_{frq}$ | $\frac{V_{sup} K_\tau \sqrt{d(1-d)}}{R \sqrt{1 + \tau_{pwm}^2 \omega_{pwm}^2}}$ | $\propto 1/f_{pwm}$ (higher = better) |
| Deadtime | $\tau_{dt}$ | $\frac{V_{sup} d K_\tau \sqrt{d_{dt}(1-d_{dt})}}{R}$ | $\propto \sqrt{d_{dt}}$ |
| Encoder phase shift | $\tau_{enc}$ | Monte Carlo simulation | Velocity-dependent |
| Cogging | $\tau_{cog}$ | RMS of measured $d_{cog,i}$ | Motor design |
| Friction | $\tau_{fr}$ | RMS of $(d_{fw,i} - d_{bw,i})/2$ | Bearings |
| Mutual | $\tau_{mtl}$ | Back-EMF vs drive waveform | Drive type |

### Optimal PWM frequency

The competing effects of $\tau_{res}$ (worse at high PWM) and $\tau_{frq}$ (worse at low PWM)
produce an optimal PWM frequency:

$$f_{pwm}^\* \approx \sqrt{\frac{R f_{clk}}{2\pi L}}$$

The paper's experimental results closely match this prediction (see Table 1 in the paper:
$f_{pwm,meas}$ vs $f_{pwm,est}$).

---

## 8. Practical Guidelines for Parameter Selection

### PWM frequency

From the model, choose PWM frequency to balance resolution ripple and frequency ripple:

- **Too low** (< ~5 kHz): Frequency ripple dominates; current lags the PWM waveform
- **Too high** (> ~20 kHz): Resolution ripple dominates; each PWM period has too few clock ticks
- **Optimal range**: ~5–15 kHz for typical hobby motors with $f_{clk} \approx 10\text{–}80$ MHz
- The optimal can be estimated from $f_{clk}$, $R$, and $L$ (see section 7)

### PWM resolution

- 8-bit (256 counts) is marginal — expect significant $\tau_{res}$
- 10-bit (1024 counts) or higher is recommended for good anticogging performance
- The paper's results at 300-count PWM (≈8.2 bit) show 49–69% peak-to-peak reduction

### Collection method selection

| Use case | Recommended method |
| :--- | :--- |
| Full rotation available | Acceleration-based (faster, continuous data) — but requires known $J$ |
| Constrained joint (< 1 rev) | Position-based (works with partial rotation) |
| Closed-loop velocity controller available | **Velocity-based PID torque measurement** (OpenFFBoard method) — no $J$ needed, no differentiation noise |
| High-resolution encoder | Either |
| Low-resolution encoder | Position-based (avoids differentiation noise) |

### Why the OpenFFBoard uses a different approach

The OpenFFBoard has a closed-loop velocity PID controller available (since it's a servo
driver), which neither Piccoli method assumes. This allows measuring torque directly from
the PID output rather than inferring it from velocity derivatives — a significant practical
advantage that eliminates both the need for $J$ and the noise amplification from
numerical differentiation.

### Minimum speed for acceleration method

Run at the lowest possible steady speed to maximize:
- **Angular displacement per encoder count** → better position resolution
- **Time per revolution** → more samples for FFT fitting
- **Minimized inertial effects** → cleaner cogging signal

The OpenFFBoard's calibration defaults (3 RPM for profile 1) follow this guideline.

---

## 9. Relation to OpenFFBoard Implementation

The OpenFFBoard's anti-cogging system implements the Piccoli method with several adaptations:

| Piccoli Paper | OpenFFBoard |
| :--- | :--- |
| **Position-based**: open-loop position hold, measure holding torque | — |
| **Acceleration-based**: open-loop constant duty cycle, differentiate velocity → $\tau = J\ddot{\theta}$ | — |
| — | **Velocity-based PID torque measurement**: closed-loop velocity PID, measure $iq_{cmd}$ output |
| CW/CCW averaging of duty cycle or acceleration | **CW/CCW averaging of DFT harmonics** (magnitude + complex-vector phase) |
| FFT fit of collected data (offline post-processing) | **Continuous Chebyshev DFT accumulation** during sweep (online) |
| Playback as lookup table or harmonic sum | **Three FF modes**: 0=harmonic sum, 1=combined bins, 2=per-direction bins |
| Deadtime/stiction separation (Algorithm 3) | **Not explicitly modeled** — absorbed by DFT (friction is near-DC, filtered by harmonic selection + CW/CCW averaging) |
| Requires $J$ for acceleration method | **No $J$ needed** — torque measured directly from PID output |
| Single RPM map | **Multi-RPM profiles** (up to 5 configurable speeds) |
| Scale/phase curves not addressed | **Scale curve + phase advance** for speed-dependent compensation |

### Key difference: Open-loop vs closed-loop measurement

The paper's two methods are both **open-loop**:

1. **Position method**: Open-loop torque command → wait for stop → measure holding torque
2. **Acceleration method**: Open-loop constant duty cycle → measure velocity ripple → differentiate

The OpenFFBoard uses a **closed-loop** method:

3. **Velocity PID method**: Close the velocity loop → command constant speed →
   measure PID output torque → that torque **is** the cogging + friction signal

This is possible because the OpenFFBoard is a servo driver with a velocity PID loop
already available. The PID output $iq_{cmd}$ represents the torque the controller must
produce to overcome disturbances (cogging + friction) and maintain constant velocity.
This is a more direct measurement than either Piccoli method: no waiting for stop,
no numerical differentiation, no need for $J$.

### Key difference: DFT replaces FFT post-processing

The paper collects raw data, then fits an FFT **offline** after collection. The OpenFFBoard
accumulates DFT coefficients **online** during the sweep using the Chebyshev recurrence,
eliminating the need to store raw position/torque data for post-processing. The averaging
principle (CW+CCW → pure cogging) is identical.

### Key difference: Phase averaging method

The paper uses direct duty-cycle averaging ($d_{cog} = (d_{max} + d_{min})/2$). The
OpenFFBoard averages in the **harmonic domain**: magnitudes are directly averaged, and
phases are averaged via complex vector conversion ($\cos\phi + i\sin\phi$ → average →
$\text{atan2}$) to avoid $\pm\pi$ wrapping issues.

---

## 10. References

1. Piccoli, M., & Yim, M. (2016). Anticogging: Torque ripple suppression, modeling, and
   parameter selection. *The International Journal of Robotics Research*, 35(1-3), 148–160.
   DOI: [10.1177/0278364915599045](https://doi.org/10.1177/0278364915599045)

2. Piccoli, M., & Yim, M. (2014). Cogging torque ripple suppression via position-based
   characterization. *Robotics: Science and Systems (RSS) Workshop on "New Generation of
   Low-Cost Actuators for High-Performance Robotics"*.

3. Armstrong, B. (1988). Friction: experimental determination, modeling and compensation.
   *IEEE International Conference on Robotics and Automation*, 1422–1427.

4. Holtz, J., & Springob, L. (1996). Identification and compensation of torque ripple in
   high-precision permanent magnet motor drives. *IEEE Transactions on Industrial Electronics*,
   43(2), 309–320.

5. Hung, J. Y., & Ding, Z. (1993). Design of currents to reduce torque ripple in brushless
   permanent magnet motors. *IEE Proceedings B - Electric Power Applications*, 140(4), 260–266.
