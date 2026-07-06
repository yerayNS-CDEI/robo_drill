# robo_drill Control UI

A PyQt5 control panel for the robo_drill mobile platform and its on-board 3-stage
linear gantry. Adapted from the arm_control UI; the UR10e arm, MoveIt planning and
GPR API tooling have been removed since this robot has no separate manipulator arm.

## Launching

```bash
ros2 run robo_drill UI
```

Requirements: `python3-pyqt5` and `QTermWidget` (`sudo apt install libqtermwidget5-0`).
A valid `ROS_DOMAIN_ID` (1–19) should be set, matching the platform launches.

## Tabs

### 1. Base Control
Brings up and manages the base platform (`mode:=base`):
- **Launch Base Robot** – `ros2 launch robo_drill platform.launch.py sim:=<mode> mode:=base controller_type:=<type> ...`
- Start Localization / Nav2 (all `mode:=base`)
- Troubleshooting: view map, list controllers, RQT, list processes
- Emergency stop, terminal + status log with filtering

### 2. Gantry Control
Jogs the on-board 3-stage linear gantry + end rotation by publishing to
`/gantry_position_controller/commands` (`std_msgs/Float64MultiArray`, order
`[gantry_z, gantry_y, gantry_x, gantry_rot]`):
- **Stage 1 – Lift / Z (m)**  – range 0.0 … 0.90
- **Stage 2 – Side / Y (m)**  – range −0.25 … 0.25
- **Stage 3 – Reach / X (m)** – range 0.0 … 0.40
- **End Rotation (rad)**      – range −π … π
- **Send Gantry Command** publishes the current slider targets; **Home (0,0,0)** re-centres.

All four stages (including the vertical Z lift, formerly the separate `column_joint`)
are one gantry driven by the single `gantry_position_controller`.

### 3. Full Control
Same platform stack as Base Control but in `mode:=full` (base + gantry brought up
together via `platform.launch.py ... mode:=full`). Includes localization/nav2,
emergency stop, troubleshooting and status log.

### 4. FSM
Launches and monitors the `task_planner_fsm` state machine (unchanged).

## Notes
- Launch/run buttons shell out to `ros2 launch|run robo_drill ...`; hover a button
  to see the exact command in its tooltip.
- The Base and Full Control tabs are mutually exclusive while a stack is running
  (starting one disables the other); the Gantry and FSM tabs stay available.
