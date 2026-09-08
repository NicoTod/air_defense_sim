# air_defense_sim

Kinematic interception simulation (counter-rocket air-defense style) for
the Robot Programming 2025/26 course — Sapienza University of Rome.

A ballistic target is fired at a city. A radar measures its position with
noise, a Kalman filter recovers the true state from the noisy measurements,
and an interceptor uses the filtered estimate to compute a collision course
and take the target down before it arrives. The interceptor does not fire
at everything: it engages only targets predicted to land on the city, and
only once the shot is actually within its fuel range. It has limited fuel
and a reload time, so the defence does not always win: roughly one threat
in ten gets through and lands among the buildings. Everything is kinematic
(no physics engine) and visualized in RViz.

## Architecture

Five independent C++ nodes, one source file each:

| Node | File | Role |
|------|------|------|
| `world_node` | `src/world_node.cpp` | Draws the static scenery: ground, the city being defended, the radar tower and the launcher pad. |
| `target_sim_node` | `src/target_sim_node.cpp` | Simulates the ground-truth ballistic trajectory, broadcasts the `world -> target` tf frame, draws the target and its trail, and blows up on the ground when it is not intercepted. Holds the sky empty for `relaunch_delay` between one target and the next. |
| `radar_sensor_node` | `src/radar_sensor_node.cpp` | Reads the true target position from tf **in its own `radar_link` frame**, adds Gaussian noise, publishes the measurement. |
| `estimator_node` | `src/estimator_node.cpp` | Transforms measurements into the `world` frame (tf2), runs a discrete Kalman filter (Eigen), predicts the future trajectory and impact point. |
| `interceptor_node` | `src/interceptor_node.cpp` | Decides whether a target is worth engaging (threat evaluation) and whether the shot is reachable, then computes the collision vector from the estimated state and flies the interceptor to the earliest reachable intercept point, within its fuel budget. |

Data flow:

```
target_sim_node ──tf──> radar_sensor_node ──/radar/measurement──> estimator_node
       ^                                                                │
       │              /estimator/pose, /estimator/velocity, /predicted_impact
       │                                                                │
       │                                                                v
       └──────────────────/interceptor/hit───────────────── interceptor_node
```

The loop is closed: when the interceptor scores a hit it publishes on
`/interceptor/hit`, and the target node sends in a fresh target instead of
letting the dead one finish its arc.

tf frames: `world` (fixed) → `target` (ground truth), `radar_link`
(static, the radar's mounting pose), `interceptor`.

Key topics:

- `/radar/measurement` (`PoseStamped`, frame `radar_link`) — noisy position
- `/estimator/pose` (`PoseStamped`, frame `world`) — filtered position
- `/estimator/velocity` (`Vector3Stamped`) — estimated velocity (never
  measured directly: recovered by the filter)
- `/estimator/predicted_impact` (`PointStamped`) — predicted landing point;
  this is what the interceptor's engage/hold-fire decision is made on
- `/interceptor/hit` (`PointStamped`) — announces a successful interception
- `/world_markers`, `/target/markers`, `/interceptor/markers`
  (`MarkerArray`), `/radar/measurement_markers`, `/estimator/markers`
  (`Marker`) — RViz visualization

## Robotics concepts demonstrated

- **tf2**: the radar measures in its local frame; the estimator transforms
  measurements into `world` before filtering (`lookupTransform` +
  `doTransform`).
- **State estimation**: textbook discrete Kalman filter (`KalmanFilter`
  class in `estimator_node.cpp`), state `[position, velocity]`, ballistic
  motion model with known gravity, white-noise-acceleration process noise.
  Includes track-loss detection with re-initialization.
- **Control / guidance**: the interceptor rolls the estimated state forward
  and steers to the earliest reachable point of the predicted trajectory,
  compensating for the age of the estimate it is flying on.
- **Decision making**: before spending a missile the launcher asks two
  questions — *is this target a threat?* (does its predicted impact fall
  within `defend_radius` of the city) and *can I even reach it?* (does the
  intercept fit in `max_flight_time`). It holds fire otherwise, waiting as
  the target closes rather than burning a missile short of it. Both
  decisions are made on the filter's estimate, never on ground truth.
- **Clean C++ / ROS 2**: one class per node, all tunables are declared
  parameters loaded from `config/params.yaml`, no global variables. The
  explosion animation is shared by two nodes through a small header
  (`include/air_defense_sim/explosion.hpp`) instead of being duplicated.

## Build and run

The workspace uses [pixi](https://pixi.sh) with RoboStack (ROS 2 Humble).
From the workspace root (`ros2_ws/`):

```bash
pixi run build   # colcon build --symlink-install
pixi run sim     # launch all five nodes
pixi run rviz    # RViz with the preconfigured view (config/sim.rviz)
```

What you see in RViz:

| Colour | Meaning |
|--------|---------|
| grey blocks | the city being defended (x ≈ 55–83) |
| teal cylinder | radar tower, at the `radar_link` frame |
| orange pad | interceptor launcher |
| **red** sphere + trail | ground-truth target |
| **white** dots | raw noisy radar measurements |
| **green** translucent sphere | Kalman estimate — deliberately drawn bigger than the target, so it reads as a cloud of belief around the truth |
| **green** line | predicted trajectory |
| **yellow** sphere | predicted impact point |
| **cyan** sphere + trail | interceptor |
| yellow→red fireball | explosion: tight and quick in the air (a kill), big and slow on the ground (a leaker) |

Every engagement is also logged:

```
[interceptor_node]: HOLD: predicted impact 21 m from the city, outside the defended zone
[interceptor_node]: WAIT: intercept needs 0.83 s of flight, fuel is 0.70 s
[interceptor_node]: HIT: target intercepted at (33.8, -2.8, 37.9)
[interceptor_node]: BURNOUT: interceptor ran out of fuel
[target_sim_node]:  IMPACT: target reached the ground at (79.0, -10.5)
```

`IMPACT` is deliberately neutral: a target reaching the ground is a defence
failure only if the interceptor engaged it. One that was held (outside the
defended zone) was *meant* to land.

## Tuning

All parameters live in `config/params.yaml` (no rebuild needed, just
relaunch). Note that a few of them are coupled and must be kept in sync:
`world_node.radar_position` with the `radar_tf` static transform in
`launch/sim.launch.py`, and `world_node.launcher_position` with
`interceptor_node.launch_position`.

The defaults are tuned so the defence stops about 90% of the targets that
actually threaten the city, with the kills happening in front of the
skyline at 20–30 m altitude. Interesting experiments:

- raise `radar_sensor_node.noise_stddev` to 2.0 and watch the filter still
  track (keep `estimator_node.measurement_stddev` matched);
- lower `estimator_node.process_noise` for a smoother but lazier estimate;
- **difficulty** is set by `interceptor_node.max_flight_time` (fuel, and so
  reach: 0.7 s at 45 m/s is ~31 m) together with `cooldown` (reload time).
  Fuel is deliberately set near the median engagement distance, so the
  close shots connect and the long ones burn out. Drop it to 0.6 and the
  hit rate falls to roughly two thirds; raise it to 0.85 and nothing ever
  gets through;
- shrink `interceptor_node.defend_radius` and watch the interceptor ignore
  more and more of the incoming targets (`HOLD` in the log) — at 0 it never
  fires at all; widen it past the skyline and it wastes missiles on targets
  that were going to fall harmlessly short;
- raise `interceptor_node.decision_delay` to see the cost of deciding on a
  filter that has not converged yet, and lower it to 0 to see wrong
  engage/hold decisions made on the filter's first, wild predictions;
- shorten `target_sim_node.relaunch_delay` to stack the engagements closer
  together, until the interceptor is still reloading when the next target
  arrives;
- widen `target_sim_node.velocity_spread` for a more chaotic threat — but
  note the targets then stop being aimed at the city, which is the point of
  the scenario;
- lower `interceptor_node.speed` until the interceptor can no longer make
  the intercept window.
