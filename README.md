# air_defense_sim

Kinematic interception simulation (counter-rocket air-defense style) for
the Robot Programming 2025/26 course — Sapienza University of Rome.

A ballistic target flies across the world. A radar measures its position
with noise, a Kalman filter recovers the true state from the noisy
measurements, and an interceptor uses the filtered estimate to compute a
collision course and take the target down. Everything is kinematic (no
physics engine) and visualized in RViz.

## Architecture

Four independent C++ nodes, one source file each:

| Node | File | Role |
|------|------|------|
| `target_sim_node` | `src/target_sim_node.cpp` | Simulates the ground-truth ballistic trajectory and broadcasts the `world -> target` tf frame. |
| `radar_sensor_node` | `src/radar_sensor_node.cpp` | Reads the true target position from tf **in its own `radar_link` frame**, adds Gaussian noise, publishes the measurement. |
| `estimator_node` | `src/estimator_node.cpp` | Transforms measurements into the `world` frame (tf2), runs a discrete Kalman filter (Eigen), predicts the future trajectory and impact point. |
| `interceptor_node` | `src/interceptor_node.cpp` | Computes the collision vector from the estimated state and flies the interceptor to the earliest reachable intercept point. |

Data flow:

```
target_sim_node ──tf──> radar_sensor_node ──/radar/measurement──> estimator_node
                                                                        │
                                              /estimator/pose, /estimator/velocity
                                                                        │
                                                                        v
                                                              interceptor_node
```

tf frames: `world` (fixed) → `target` (ground truth), `radar_link`
(static, the radar's mounting pose), `interceptor`.

Key topics:

- `/radar/measurement` (`PoseStamped`, frame `radar_link`) — noisy position
- `/estimator/pose` (`PoseStamped`, frame `world`) — filtered position
- `/estimator/velocity` (`Vector3Stamped`) — estimated velocity (never
  measured directly: recovered by the filter)
- `/estimator/predicted_impact` (`PointStamped`) — predicted landing point
- `*/markers` — RViz visualization for every node

## Robotics concepts demonstrated

- **tf2**: the radar measures in its local frame; the estimator transforms
  measurements into `world` before filtering (`lookupTransform` +
  `doTransform`).
- **State estimation**: textbook discrete Kalman filter (`KalmanFilter`
  class in `estimator_node.cpp`), state `[position, velocity]`, ballistic
  motion model with known gravity, white-noise-acceleration process noise.
  Includes track-loss detection with re-initialization.
- **Control / guidance**: the interceptor rolls the estimated state forward
  and steers to the earliest reachable point of the predicted trajectory.
- **Clean C++ / ROS 2**: one class per node, all tunables are declared
  parameters loaded from `config/params.yaml`, no global variables.

## Build and run

The workspace uses [pixi](https://pixi.sh) with RoboStack (ROS 2 Humble).
From the workspace root (`ros2_ws/`):

```bash
pixi run build   # colcon build --symlink-install
pixi run sim     # launch all four nodes
pixi run rviz    # RViz with the preconfigured view (config/sim.rviz)
```

In RViz you will see:

- **orange sphere** — ground-truth target
- **red dots** — noisy radar measurements
- **green sphere + line** — Kalman estimate and predicted trajectory
- **yellow sphere** — predicted impact point
- **blue sphere** — interceptor (white flash on interception)

Interceptions are also logged by `interceptor_node`:

```
[interceptor_node]: Target intercepted at (23.4, -0.3, 43.2)
```

## Tuning

All parameters live in `config/params.yaml` (no rebuild needed, just
relaunch). Interesting experiments:

- raise `radar_sensor_node.noise_stddev` to 2.0 and watch the filter still
  track (keep `estimator_node.measurement_stddev` matched);
- lower `estimator_node.process_noise` for a smoother but lazier estimate;
- lower `interceptor_node.speed` until the interceptor can no longer make
  the intercept window.
