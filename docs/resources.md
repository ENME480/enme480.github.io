---
title: Resources
description: Setup guides, references, and external documentation for ENME480
---

<p class="eyebrow">ENME480 · Reference</p>

# Resources

<p class="lede">Setup guides for the toolchain, references for the math, and the handful of external docs worth bookmarking.</p>

## Course documents

<dl class="spec" markdown>
<dt>Syllabus</dt>
<dd markdown="span"><a href="assets/docs/syllabus-fall2026.pdf">Fall 2026 syllabus (PDF)</a> — grading, policies, week-by-week outline</dd>
<dt>UR3e DH table</dt>
<dd markdown="span"><a href="assets/docs/dh_table.pdf">Denavit-Hartenberg parameters (PDF)</a></dd>
<dt>FK datasheet</dt>
<dd markdown="span"><a href="assets/docs/fk_datasheet.pdf">Forward kinematics datasheet (PDF)</a></dd>
<dt>Camera notes</dt>
<dd markdown="span"><a href="assets/docs/PerspectiveTransformEstimation.pdf">Perspective transform estimation (PDF)</a></dd>
</dl>

## Setup guides

Work through these in order the first time. Everything after Week 1 assumes they are done.

<div class="grid cards" markdown>

-   :material-ubuntu:{ .lg .middle } **Ubuntu**

    ---

    Installing 22.04, dual boot and VM options, partitioning, drivers.

    [Ubuntu Setup](ubuntu-setup.md)

-   :material-robot-outline:{ .lg .middle } **ROS 2**

    ---

    Installing Humble, sourcing the environment, building a workspace.

    [ROS Setup](ros-setup.md)

-   :material-cube-outline:{ .lg .middle } **Gazebo**

    ---

    Launching the simulated UR3e cell and troubleshooting the usual failures.

    [Gazebo Setup](gazebo-setup.md)

-   :material-laptop:{ .lg .middle } **Dev environment**

    ---

    VS Code against the course Docker image, and a terminal setup that will not fight you.

    [Dev Environment](dev-environment.md)

</div>

## Language and tooling

<div class="grid cards" markdown>

-   :material-language-python:{ .lg .middle } **Python**

    ---

    The subset used in lab: NumPy arrays, classes, callbacks, and the ROS 2 client library.

    [Python Basics](python-basics.md)

-   :material-git:{ .lg .middle } **Git**

    ---

    Cloning lab code, branching, and getting out of the states you will end up in.

    [Git Basics](git-basics.md)

-   :material-function-variant:{ .lg .middle } **Kinematics**

    ---

    DH conventions, homogeneous transforms, the UR3e parameter table, FK and IK derivations.

    [Kinematics Reference](kinematics-reference.md)

</div>

## External documentation

| Source | Use it for |
|---|---|
| [ROS 2 Humble docs](https://docs.ros.org/en/humble/index.html) | Concepts, tutorials, `rclpy` API |
| [Gazebo docs](https://gazebosim.org/docs) | Worlds, plugins, spawning models |
| [NumPy reference](https://numpy.org/doc/stable/reference/) | Array and linear algebra operations |
| [OpenCV Python tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) | ArUco detection, perspective warps |
| [Universal Robots UR3e](https://www.universal-robots.com/products/ur3-robot/) | Hardware specifications and reach |
| [Robotics Stack Exchange](https://robotics.stackexchange.com/) | Where ROS Answers questions live now |
| [UMD Robotics Minor](https://robotics.umd.edu/minor) | The minor this course leads into |

## Course repositories

| Repository | Contents |
|---|---|
| [ENME480/Lab-Code](https://github.com/ENME480/Lab-Code) | Lab handouts and starter packages |
| [ENME480/enme480_project](https://github.com/ENME480/enme480_project) | Final project package with redacted scripts |
| [github.com/ENME480](https://github.com/ENME480) | Everything else |

## Commands worth memorizing

```bash
# ROS 2 — source before anything else, in every new terminal
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Inspect a running system
ros2 node list
ros2 topic list
ros2 topic echo /joint_states

# Build only the package you changed
cd ~/ros2_ws && colcon build --packages-select <pkg> --symlink-install
```

!!! tip "When something breaks after a rebuild"
    Re-source `install/setup.bash`. A stale environment explains most "my node disappeared" reports.
