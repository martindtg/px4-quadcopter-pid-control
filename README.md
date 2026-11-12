# Quadcopter Attitude PID Control: Laplace Pole-Placement & PX4 Flight Analysis

**Author:** Martin Dutang — *CentraleSupélec & McGill University (Bachelor of Global Engineering)*  
**Stack:** `C++17` · `Python` · `PX4 Autopilot` · `Pixhawk 2.4.8` · `QGroundControl`

---

## Overview

Originally motivated by winning the **Imperial College Innovation Challenge** with an autonomous rural medical-delivery UAV concept, this project combines:
1. **Scratch-built quadcopter hardware** (`B-CUBE LX350 Pro` frame, `Pixhawk 2.4.8`, `2212 920KV` brushless motors, `BLHeli 30A` ESCs, `4S 5200mAh 60C` LiPo, GPS/compass + telemetry radio).
2. **Analytical Laplace-domain control design** modeling single-axis rotational dynamics ($I\ddot{\theta} + b\dot{\theta} = \tau$) to derive closed-loop gains via second-order pole placement ($\zeta = 0.7$, $\omega_n = 5.0\text{ rad/s}$).
3. **500 Hz C++17 RK4 attitude simulation** (`src/simulate_attitude.cpp`) with integral anti-windup and low-pass derivative filtering under external wind-gust torque disturbances.
4. **Empirical PX4 flight-log (`.ulg`) benchmarking** across **9 outdoor flight trials** comparing *Manual*, *Stabilized*, and *Autonomous Mission* modes with propagated GPS uncertainty bounds.

| Custom Quadcopter Prior to Takeoff | Outdoor Autonomous Flight Trial |
| :---: | :---: |
| ![Quadcopter Ground](assets/quadcopter_ground.jpg) | ![Quadcopter Flight](assets/quadcopter_flight.jpg) |

---

## Mathematical Model & Gain Derivation

Approximating each rotational axis (pitch/roll) with motor mass $m = 0.055\text{ kg}$ at moment arm $d = 0.225\text{ m}$:

$$I = 2md^2 = 2(0.055)(0.225)^2 = 0.00557\text{ kg}\cdot\text{m}^2, \qquad b \approx 0.001\text{ N}\cdot\text{m}\cdot\text{s/rad}$$

Applying the Laplace transform to $I\ddot{\theta}(t) + b\dot{\theta}(t) = \tau(t)$ yields the open-loop plant transfer function:

$$G_{\text{plant}}(s) = \frac{\Theta(s)}{T(s)} = \frac{1}{Is^2 + bs}$$

Matching the closed-loop PD characteristic polynomial $s^2 + \frac{b + K_d}{I}s + \frac{K_p}{I} = 0$ against the canonical second-order target $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$ ($\zeta = 0.7$, $\omega_n = 5.0\text{ rad/s}$):

$$K_p = I\omega_n^2 = 0.1393\text{ N}\cdot\text{m/rad}, \qquad K_d = 2\zeta\omega_n I - b = 0.0380\text{ N}\cdot\text{m}\cdot\text{s/rad}$$

![Closed-Loop Step Response](assets/pid_laplace_step_response.png)

---

## Empirical PX4 Outdoor Flight Results (9 Logged `.ulg` Trials)

Across three 25 m corridor trials per flight mode, full autonomous **Mission PID** reduced sample trajectory standard deviation by **11.6×** compared to Manual mode ($\sigma = 0.30\text{ m}$ vs. $3.47\text{ m}$) and cut pitch steady-state error from **$-60\%$** (Manual) / **$-92\%$** (Stabilized) down to **$-15\%$** ($+10\%$ overshoot, $0.03\text{ s}$ settling time):

![PX4 Flight Analysis](assets/px4_flight_trials_analysis.png)

---

## Quick Start

```bash
# Compile C++17 500 Hz RK4 simulator & regenerate all figures
make run
```

## Repository Structure

```text
px4-quadcopter-pid-control/
├── include/pid_controller.hpp      # Discrete PID w/ trapezoidal anti-windup & LPF derivative
├── src/simulate_attitude.cpp       # 500 Hz RK4 rotational plant & wind-gust benchmark
├── python/analyze_px4_modes.py     # Flight-log (.ulg) metrics & matplotlib visualization
├── data/px4_flight_trials.csv      # Empirical data from 9 outdoor PX4 flights
├── assets/                         # Hardware photos & generated performance plots
└── Makefile
```
