# nav2_sean

Ubuntu 22.04 / ROS 2 Humble port of `px4_nav2_bridge`. Nav2-to-PX4 Offboard
bridge for indoor (no-GPS, no map_server/AMCL) minimal navigation testing,
same architecture as the original: `px4_odom_tf` (PX4 odometry -> Nav2
TF/`/odom`) + `cmd_vel_bridge` (Nav2 `/cmd_vel` -> PX4 Offboard setpoints) +
a trimmed-down Nav2 stack (`bridge_launch.py`).

## What changed from px4_nav2_bridge

1. **Coordinate convention**: `map.x = e, map.y = n, map.z = -d` (standard
   ENU — East/North/Up), instead of the original's
   `map.x = n, map.y = -e, map.z = -d` (aligned to how the OptiTrack rig
   happened to be placed in the room). Affects `px4_odom_tf.py`'s published
   TF/`/odom` position+velocity, and `cmd_vel_bridge.py`'s `apply_fence()`
   NED<->map conversion for the geofence. Nothing else changed in those two
   files' logic.
2. **Heading/orientation: left exactly as-is, on purpose** — see below.
   `px4_nav2_bridge`'s heading handling is still unresolved (Motive rigid
   body local-axis bug, see "Known issue" below); we're holding off changing
   it until that's discussed further.
3. **`use_stamped` default flipped `True` -> `False`** in `cmd_vel_bridge.py`
   — see "Jazzy -> Humble gotcha" below. This one actually matters
   functionally for the port, unlike the coordinate change.
4. **`nav2_navfn_planner`'s plugin name changed from `::` to `/`** in
   `nav2_params.yaml`: `planner_server` failed to bring up with `Failed to
   create global planner. ... class nav2_navfn_planner::NavfnPlanner ...
   does not exist. Declared types are nav2_navfn_planner/NavfnPlanner ...`.
   **This is not a blanket Nav2-version convention** — don't assume every
   `plugin:` line needs the same treatment. Converting all of them to `/`
   on a hunch immediately broke `nav2_costmap_2d::InflationLayer` and
   `nav2_controller::SimpleProgressChecker`/`SimpleGoalChecker`, which
   turned out to still be registered with `::` on this same Humble
   install — the error message named the actual declared types each time,
   confirming it. So only `nav2_navfn_planner` is `/`; everything else
   (behavior_server's 4 behaviors, bt_navigator's 2 navigators, the
   progress/goal checkers, MPPI controller, smoother, both costmaps'
   inflation layer, waypoint_follower) is back to the original `::` from
   `px4_nav2_bridge`, unconfirmed either way — if any of those fail to
   bring up with the same "class ... does not exist, declared types are
   ..." error, fix only that one line to match what the error says, don't
   generalize again.

## Where heading currently comes from (unchanged from px4_nav2_bridge)

Two different, **inconsistent** sources, kept as-is for now:

| File | What it does with heading | Source |
|---|---|---|
| `px4_odom_tf.py` (feeds Nav2's TF/`/odom` — what costmap/MPPI actually see) | Hardcoded to identity (`yaw=0`), never reads anything | none |
| `cmd_vel_bridge.py`'s `on_odom()` (only used to fill `TrajectorySetpoint.yaw` while holding position, i.e. when not receiving live Nav2 `/cmd_vel`) | Computed via `atan2` from the quaternion | `VehicleOdometry.q` on `<namespace>/fmu/out/vehicle_odometry` |

Since `hover_test`'s real-flight testing is working and already has a
working relationship with `vehicle_local_position_v1.heading` + PX4's EKF2
(`EKF2_EV_CTRL` bitmask, magnetometer-based heading, etc. — see that
project's notes), it's worth revisiting this project's heading handling
against what's now known, once we decide what to do about the Motive
rigid-body issue below.

### Known issue (inherited, not fixed)

Motive (OptiTrack)'s rigid body local-axis definition for this airframe is
wrong: rotating the physical nose horizontally shows up as a change in
**roll** in Motive, not yaw. So any yaw derived from the mocap-fed
`VehicleOdometry.q` is untrustworthy. Because of that, `px4_nav2_bridge`
(and this port) made a set of deliberate compromises, all still in place:

- `px4_odom_tf.py` publishes identity orientation instead of a real one.
- `cmd_vel_bridge.py`'s `convert()` skips the body->world yaw rotation
  entirely (treats body frame = world frame).
- `cmd_vel_bridge.py`'s `enable_yaw` parameter defaults to `False` — Nav2's
  turn commands aren't forwarded to PX4, because Nav2 would think it's not
  turning (since `px4_odom_tf` always reports yaw=0) and spin forever.
- `nav2_params.yaml` disables every MPPI critic that scores based on
  orientation (`GoalAngleCritic`, `PathAngleCritic`, `PreferForwardCritic`)
  and sets `yaw_goal_tolerance` to ~π (accept any final heading).

None of this is touched in this port — it's carried over unchanged until
we decide how to fix it.

## Jazzy -> Humble gotcha: `use_stamped` / `enable_stamped_cmd_vel`

This is the one change in this port that actually affects whether the
bridge works at all, not just a coordinate convention:

- `px4_nav2_bridge` was tuned against a specific Jazzy Nav2 install where
  `enable_stamped_cmd_vel` defaulted to `false` (confirmed via
  `ros2 param get`) — meaning `controller_server`/`behavior_server`/
  `velocity_smoother` published plain `geometry_msgs/Twist`. Since
  `cmd_vel_bridge.py` subscribes to `TwistStamped` by default
  (`use_stamped=True`), the original project explicitly set
  `enable_stamped_cmd_vel: true` in `nav2_params.yaml` to make the types
  match.
- **A type mismatch here doesn't error** — DDS just never connects the
  publisher/subscriber pair, so `/cmd_vel` silently receives nothing,
  which looks identical to "robot just isn't moving."
- Humble's Nav2 is a different, older version; it may not even have an
  `enable_stamped_cmd_vel` parameter (stamped `cmd_vel` was added later).
  So this port flips `cmd_vel_bridge.py`'s `use_stamped` default to
  `False`, matching Humble's more likely default of plain `Twist`.
- **This needs re-verification on the actual Humble machine** — run
  `ros2 param get /controller_server enable_stamped_cmd_vel` (if it exists
  at all) the same way the original project did on Jazzy, and make sure
  `nav2_params.yaml`'s `enable_stamped_cmd_vel` lines agree with whatever
  `cmd_vel_bridge.py`'s `use_stamped` ends up being — they must match each
  other, not any particular hardcoded value.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/shared/ros2_ws   # or wherever this workspace lives on the 22.04 machine
colcon build --packages-select nav2_sean
source install/setup.bash
```

## Run (adapted from px4_nav2_bridge's simulation command sheet)

```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Nav2 + bridge
ros2 launch nav2_sean bridge_launch.py \
  topic_odometry:=/MAV1/fmu/out/vehicle_odometry \
  topic_offboard_mode:=/MAV1/fmu/in/offboard_control_mode \
  topic_setpoint:=/MAV1/fmu/in/trajectory_setpoint \
  topic_command:=/MAV1/fmu/in/vehicle_command \
  target_alt:=0.5

# Takeoff
ros2 service call /cmd_vel_bridge/start std_srvs/srv/Trigger

# Move towards a goal, map ENU: +x=east, +y=north
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}" --feedback

# Land
ros2 service call /cmd_vel_bridge/land std_srvs/srv/Trigger
```

(`topic_odometry` etc. default to unnamespaced `/fmu/...` — override to
match whatever namespace the target vehicle's uXRCE-DDS client actually
uses, same as `hover_test`'s `namespace` parameter. The example above uses
`/MAV1`, matching the real vehicle `hover_test` currently flies on.)

## Test

```bash
colcon test --packages-select nav2_sean
colcon test-result --verbose
```
