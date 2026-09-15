---
title: Gazebo in this course
description: Gazebo ships with the course Docker image. What is in it, how to launch the course simulations, and how to drive the GUI.
---

<p class="eyebrow">ENME480 · Wiki</p>

# Gazebo in this course

<p class="lede">You do not install Gazebo. It is already in the course Docker image.</p>

!!! danger "Do not run `sudo apt install gazebo`"
    That installs **Gazebo Classic** (`gazebo11`), which is a different,
    older simulator from the one this course uses. Installing it gives you two
    simulators fighting over the same ROS topics and model paths, and the
    failures are confusing — worlds that will not load, plugins that are not
    found, robots that appear but do not move.

    The image ships **Gazebo (GZ, formerly Ignition)** with the `ros_gz` bridge.
    That is what every lab uses.

## What the image gives you

Installed as part of the course image, nothing for you to add:

| | |
|---|---|
| **Simulator** | Gazebo (GZ), launched as `ign gazebo` |
| **ROS 2 bridge** | `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim` |
| **Robot** | Universal Robots description and UR3e MoveIt config |
| **Course packages** | `enme480_sim`, `ur3e_mrc_sim` (from the course repo) |

## Checking it runs

Inside the container:

```bash
ign gazebo
```

A window should open with a list of example worlds. That is the whole check —
close it again.

If no window appears, the problem is the container reaching your display, not
Gazebo. See the troubleshooting section in [Ubuntu Setup](ubuntu-setup.md), and
confirm you started the container with the right compose file for your machine.

## Launching the course simulation

You will not write your own world files. The labs use prepared launch files.
From [Week 6](labs/week-06.md) onward, in separate terminals or `tmux` panes:

```bash
# the UR3e in Gazebo
ros2 launch enme480_sim enme480_ur3e_sim.launch.py

# the simulated robot's control interface
ros2 launch ur3e_mrc_sim ur3e_enme480.launch.py

# the ENME480 command layer
ros2 launch ur3e_enme480 ur3e_sim_enme480.launch.py
```

Then commands go to the robot over a topic, exactly as they do on the real arm:

```bash
ros2 topic pub --once /ur3e/command ur3e_mrc_msgs/msg/CommandUR3e "destination: [0, -1.57, -1.57, 0, 0, 0]
v: 1.0
a: 1.0
io_0: false"
```

The point of the simulator in this course is that the **same topic interface**
drives the real UR3e. Code that works in Gazebo should work on the arm, which is
why we develop in simulation first.

!!! tip "Use tmux for the panes"
    The image has `tmux` configured. `tmux` to start, `Ctrl+A b` to split
    horizontally, `Ctrl+A v` to split vertically. Much easier than juggling four
    `connectToDocker.sh` terminals.

## Driving the GUI

### Camera

| Action | Control |
|---|---|
| Orbit | Left click and drag |
| Pan | Middle click and drag, or Shift + left drag |
| Zoom | Scroll wheel |
| Look at a model | Double click it |

### Simulation

| Action | Control |
|---|---|
| Play / pause | Buttons at the bottom left |
| Step one frame | Step button, while paused |
| Reset | World menu, Reset |

Pausing is genuinely useful. If a robot is moving unexpectedly, pause and step
through rather than trying to read a blur.

## Troubleshooting

**No window opens.** The container cannot reach your display. This is the same
problem `rqt` has, and it is covered in
[Ubuntu Setup](ubuntu-setup.md). Check you used the right compose file for your
machine (NVIDIA or standard).

**Very slow, or a black window.** Usually software rendering because the GPU is
not available to the container. On a machine with an NVIDIA GPU, confirm you did
the NVIDIA step in Ubuntu Setup. In a VM it will be slow regardless — reduce the
window size and expect low frame rates.

**The robot appears but does not move.** The simulator is running but the
control layer is not. Check all three launch files above are up, then:

```bash
ros2 topic list
ros2 topic info /ur3e/command
```

If the publisher or subscriber count is 0, one of the launch files is not
running or died on startup. Read its terminal.

**A launch file is not found.** Your workspace is not sourced, or the package is
not built:

```bash
cd ~/enme480_ws
colcon build --symlink-install
source install/setup.bash
```

Remember that a terminal opened before a build does not know about it.

## Getting help

- **Gazebo docs**: [gazebosim.org/docs](https://gazebosim.org/docs)
- **ros_gz**: [github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
- **Piazza**, office hours, and lab sessions for anything course-specific

## Next

1. [ROS 2 in the Container](ros-setup.md) — the commands you will use alongside it
2. [Week 4 Lab](labs/week-04.md) — first Gazebo studio
