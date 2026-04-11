# Sensor Fusion System — Analysis & Optimisation Report

**Date:** 2026-04-11  
**Scope:** All source files in `sensor_fusion_system-main/`  
**Severity key:** 🔴 Critical bug · 🟠 Logic error · 🟡 Correctness concern · 🔵 Optimisation · ⚪ Minor / style

---

## Executive Summary

Seven distinct issues were found across the codebase — two of which silently corrupt filter state or permanently latch the safety system into a failed state. All issues have been fixed in-place. The changes do not alter the external API (topics, services, or launch files).

---

## Issues Found & Fixed

### 1. 🔴 `fusion_node.cpp` — IMU observation matrix H has wrong dimensions

**File:** `src/fusion/fusion_node.cpp`  
**Original lines:** 108–122

**Problem:** `IMU_MEAS_DIM` was `3`, so the measurement vector was `[ax, ay, ω]`. The observation matrix H was 3 × 6 with only `H(2,5) = 1.0` set; rows 0 and 1 were all zeros. This means the innovation in dimensions 0 and 1 was `[ax − 0, ay − 0]` — raw acceleration values treated as if they should be zero in the state. The resulting Kalman gain then pulled every state component toward incorrect values on every IMU message, silently corrupting position and velocity estimates.

**Root cause:** Linear accelerations (`ax`, `ay`) are not direct state observables in a linear constant-velocity model. The state vector is `[x, y, θ, vx, vy, ω]`; accelerations require double integration (nonlinear) and belong in the prediction step of a proper EKF control input, not in the measurement update.

**Fix:** Reduced `IMU_MEAS_DIM` to `1`. The IMU callback now only measures angular velocity ω (gyro-z), which maps directly and correctly to `state[5]` via `H(0,5) = 1.0`.

---

### 2. 🟠 `fusion_node.cpp` — Covariance update is numerically unstable

**File:** `src/fusion/fusion_node.cpp`  
**Original lines:** 121–122, 183–184

**Problem:** Both `imuCallback` and `lidarCallback` used the simple form:
```
P = (I − KH) * P
```
This is mathematically equivalent to the Joseph form only when K is exactly optimal. In practice, floating-point rounding makes K slightly sub-optimal, and the simple form can produce a non-symmetric, non-positive-definite covariance matrix — causing the filter to diverge over time.

**Fix:** Replaced with the Joseph (symmetric) form in both callbacks:
```
P = (I − KH) * P * (I − KH)ᵀ + K * R * Kᵀ
```
This guarantees positive semi-definiteness regardless of numerical precision.

---

### 3. 🟠 `fusion_node.cpp` — `normalizeAngle` uses an unbounded while-loop

**File:** `src/fusion/fusion_node.cpp`  
**Original lines:** 211–216

**Problem:** The original implementation looped repeatedly adding or subtracting `2π`. For angles far from `[−π, π]` (e.g., after many integration steps), this could execute O(n) iterations.

**Fix:** Replaced with `std::remainder(angle, 2π)`, which is O(1) and defined by IEEE 754 to return a value in `(−π, π]`.

---

### 4. 🟡 `fusion_node.cpp` — Artificial noise injected inside the fusion node

**File:** `src/fusion/fusion_node.cpp`  
**Original lines:** 165–167

**Problem:** The lidar callback added Gaussian position noise *inside the fusion node* using its own RNG. Noise simulation belongs in the sensor node (`lidar_sensor_node.cpp` already does this). Adding noise again in the fusion node double-counts sensor uncertainty and makes the filter's measurement noise model `R` incorrect relative to actual incoming data.

**Fix:** Removed the noise injection and the associated `rng_` / `pos_noise_` members entirely.

---

### 5. 🟡 `fusion_node.cpp` — `prediction_rate` parameter declared in `params.yaml` but never read

**File:** `src/fusion/fusion_node.cpp`  
**Original line:** 46 (hardcoded `20ms`)

**Problem:** `params.yaml` declares `prediction_rate: 50.0` for the fusion node, but the timer was hardcoded to `20ms` without reading the parameter — so the config file had no effect.

**Fix:** Added `declare_parameter("prediction_rate", 50.0)` and `get_parameter(...)`. The timer period is now derived from the parameter value.

---

### 6. 🟠 `safety_monitor_node.cpp` — Health flag never recovers; latches permanently to `false`

**File:** `src/safety/safety_monitor_node.cpp`  
**Original line:** 43–46

**Problem:** `system_healthy_ = false` was set when a fault was detected, but it was never reset to `true`. Once any single bad pose or covariance reading arrived, the safety service would report "System has issues" forever — even after the system fully recovered. This makes the monitor useless for actual safety decision-making.

**Fix:** Split into two independent flags (`pose_healthy_`, `imu_healthy_`). Each check now re-evaluates on every message and sets the flag to `true` when the condition clears. `isSystemHealthy()` returns their conjunction. The service response also reports *which* subsystem faulted.

---

### 7. 🟠 `safety_monitor_node.cpp` — IMU acceleration check logs a warning but does not mark the system unhealthy

**File:** `src/safety/safety_monitor_node.cpp`  
**Original lines:** 56–64

**Problem:** `checkImuSafety` logged a warning for unrealistic acceleration (`|a| > 50 m/s²`) but never updated `system_healthy_`, making the warning purely cosmetic and the safety service unaware of the fault.

**Fix:** `checkImuSafety` now sets `imu_healthy_ = false` on detection and restores it to `true` when readings return to normal.

---

### 8. 🟡 `lidar_sensor_node.cpp` — `scan_time` hardcoded to `1/10` regardless of `scan_rate` parameter

**File:** `src/sensors/lidar_sensor_node.cpp`  
**Original line:** 57

**Problem:** `msg->scan_time = 1.0 / 10.0;` is hardcoded. If `scan_rate` is changed via `params.yaml`, the published scan metadata reports the wrong scan period, which can confuse downstream scan-matching or temporal synchronisation.

**Fix:** Stored `scan_rate` as a member `rate_` and set `msg->scan_time = 1.0 / rate_`.

---

### 9. 🟡 `CMakeLists.txt` — Test executables not registered; GTest tests never run

**File:** `CMakeLists.txt`  
**Original lines:** 93–96

**Problem:** The `BUILD_TESTING` block called `ament_lint_auto_find_test_dependencies()` but never called `ament_add_gtest()` for either C++ test file. Running `colcon test` would trigger only lint checks; the Google Test suites in `test/cpp/` were compiled but silently discarded.

**Fix:** Added `ament_add_gtest(test_fusion ...)` and `ament_add_gtest(test_imu_sensor ...)` with appropriate Eigen and rclcpp linkage. Also removed `find_package(std_msgs)` — `std_msgs` is not used by any target in this package.

---

### 10. 🟡 `params.yaml` — Missing `angle_increment` for lidar node

**File:** `config/params.yaml`

**Problem:** `lidar_sensor_node` declares `angle_increment` as a parameter (defaulting to `π/180 ≈ 0.01745 rad`), but the config file did not set it. Any deployment relying on config-file control could not change this value without recompiling.

**Fix:** Added `angle_increment: 0.01745329` (≈ 1°) to the lidar section of `params.yaml` with an explanatory comment.

---

### 11. 🔵 `test_imu_sensor.cpp` — `rclcpp::init` / `shutdown` called inside a test case

**File:** `test/cpp/test_imu_sensor.cpp`  
**Original lines:** 26, 34

**Problem:** `rclcpp::init(0, nullptr)` and `rclcpp::shutdown()` were called inside `TEST(ImuSensorTest, MessageHasCorrectFields)`. If any subsequent test needed rclcpp, the context would already be shut down. The second test (`CovarianceIsSet`) ran without an active rclcpp context.

**Fix:** Introduced a `::testing::Environment` subclass (`RclcppEnvironment`) that calls `init` in `SetUp` and `shutdown` in `TearDown`, registered via `AddGlobalTestEnvironment`. This ensures rclcpp is available to all tests and shut down cleanly once.

---

## Summary Table

| # | File | Severity | Issue | Fixed |
|---|------|----------|-------|-------|
| 1 | `fusion_node.cpp` | 🔴 Critical | IMU H-matrix zeros corrupt state on every IMU message | ✅ |
| 2 | `fusion_node.cpp` | 🟠 Logic | Simple covariance form — numerically unstable | ✅ |
| 3 | `fusion_node.cpp` | 🔵 Perf | `normalizeAngle` while-loop — unbounded iterations | ✅ |
| 4 | `fusion_node.cpp` | 🟡 Correctness | Noise injected inside fusion node — double-counts R | ✅ |
| 5 | `fusion_node.cpp` | 🟡 Correctness | `prediction_rate` param ignored — timer hardcoded | ✅ |
| 6 | `safety_monitor_node.cpp` | 🟠 Logic | Health flag latches `false` permanently — no recovery | ✅ |
| 7 | `safety_monitor_node.cpp` | 🟠 Logic | IMU fault detected but `system_healthy_` not updated | ✅ |
| 8 | `lidar_sensor_node.cpp` | 🟡 Correctness | `scan_time` hardcoded to 0.1 s regardless of rate | ✅ |
| 9 | `CMakeLists.txt` | 🟡 Correctness | GTest targets not registered — tests never ran | ✅ |
| 10 | `params.yaml` | ⚪ Minor | `angle_increment` missing from lidar config | ✅ |
| 11 | `test_imu_sensor.cpp` | ⚪ Minor | `rclcpp::init/shutdown` inside individual test case | ✅ |

---

## Recommendations for Future Improvement

These are out of scope for this optimisation pass but worth addressing before production use.

**Replace centroid-based lidar with a scan-matcher.** Computing the centroid of all laser returns gives the average of obstacle reflections in sensor frame, not the robot's position. Integrate [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) or [cartographer](https://google-cartographer.readthedocs.io/) to get a proper map-frame pose from lidar.

**Upgrade to a true EKF.** Linear accelerations (ax, ay) from the IMU are currently unused. A proper Extended Kalman Filter prediction step can incorporate them as control inputs (`u = [ax, ay]ᵀ`) in a nonlinear motion model, significantly improving velocity and position estimates between lidar updates.

**Add thread safety for MultiThreadedExecutor.** `pose_healthy_` and `imu_healthy_` in `SafetyMonitorNode`, and `state_` / `covariance_` in `FusionNode`, are accessed from multiple callbacks. With the default `SingleThreadedExecutor` this is safe, but switching to `MultiThreadedExecutor` requires wrapping shared state in `std::atomic` or a `std::mutex` guard.

**Expand test coverage.** Current tests only verify message field values. Priority additions: Kalman filter convergence test (verify state approaches true value after N updates), covariance positive-definiteness assertion after many update steps, safety monitor state-machine tests (fault → recovery cycle).

**Validate quaternion output.** `publishPose` sets only `orientation.z` and `orientation.w`, leaving `orientation.x = 0, orientation.y = 0`. This is correct for a 2-D planar robot but should be asserted explicitly or documented.
