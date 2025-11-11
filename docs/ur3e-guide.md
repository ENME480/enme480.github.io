# UR3e Quick Reference

Hands-on notes for operating the UR3e in ENME480 labs. Use this as a quick refresher before robot sessions or while debugging in the lab.

## Powering on and arming
- Follow the detailed walk-through in [Week 05 — UR3e Intro](labs/week-05.md#powering-on-the-ur3e) for pictures of the power and arm buttons.
- If the arm does not respond after powering on, press the arm button twice—the first press powers the robot, the second releases the brakes.
- Keep the physical e-stop within reach; if it has been pressed, twist to release before trying to arm the robot.

## Operating modes
- **Freedrive:** Hold the freedrive button on the back of the teach pendant to move the arm manually. Use this to get joint guesses before running code.
- **Program mode:** Run scripts from the teach pendant or from `ur_robot_driver` in ROS 2. Confirm the controller is in *Remote Control* before launching external programs.
- **Speed slider:** Start at 10–20% for first runs; increase only after verifying the trajectory.

## Working with ROS 2
- Source your ROS 2 workspace, then use `ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=ROBOT_IP`.
- Use `ros2 topic echo /joint_states` to confirm feedback and `rviz2` to monitor the robot model.
- Reference [Week 09 — IK Lab](labs/week-09.md) for the full ROS 2 driver workflow.

## Calibration & coordinate frames
- Ensure the base frame is aligned with the cell layout; if the table was moved, redo the feature alignment process from Week 05.
- Use the provided calibration blocks and checkerboards; see the perspective PDF in [Week 11](labs/week-11.md#camera-perspective-pdf) for camera-to-base alignment steps.

## Safety checklist
- Keep the work envelope clear and warn teammates before executing motion.
- Never bypass safety interlocks or modify the protective stops.
- If anything looks wrong, hit the e-stop first, then diagnose.

## Troubleshooting tips
- **Robot won’t arm:** verify e-stop released, confirm mode selector is in *Auto*, then reboot the controller if needed.
- **Driver disconnects:** check Ethernet cable, confirm the control box shows *Remote Control*, and restart the ROS 2 driver node.
- **Unexpected motion:** immediately press e-stop, inspect the URScript/ROS command, and test at a low speed.

Need deeper theory? See [Kinematics Reference](kinematics-reference.md) and the lab handouts embedded throughout the weekly pages.
