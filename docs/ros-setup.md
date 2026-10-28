---
title: ROS 2 in this course
description: You do not install ROS 2 for ENME480 - it comes with the Docker image. What is in the image, how to check it works, and the commands you will use.
---

<p class="eyebrow">ENME480 · Wiki</p>

# ROS 2 in this course

<p class="lede">You do not install ROS 2. It is already in the course Docker image.</p>

!!! warning "Do not follow a ROS 2 installation guide"
    Every ROS 2 tutorial online starts by telling you to install ROS 2 Humble on
    Ubuntu. **Skip that.** Installing ROS 2 on your own machine on top of the
    course image gives you two copies that shadow each other, and the symptoms
    are miserable to debug — packages that exist but will not import, nodes that
    cannot see each other, sourcing that silently picks the wrong one.

    Everything you need is inside the container. If a guide says
    `sudo apt install ros-humble-desktop`, you are on the wrong page.

## Where ROS 2 comes from

The image built in [Ubuntu Setup](ubuntu-setup.md) is based on
`osrf/ros:humble-desktop` and already contains:

| | |
|---|---|
| **ROS 2** | Humble Hawksbill, full desktop install |
| **Build tools** | `colcon`, `rosdep`, `vcstool` |
| **Simulation** | Gazebo (GZ), `ros_gz` bridge, `turtlesim` |
| **GUI tools** | `rqt` and its plugins, `rviz2` |
| **Robot drivers** | Universal Robots driver and UR3e MoveIt config |
| **Extras** | `tf_transformations`, `usb-cam`, MoveIt |

So your job is to get **into** the container, not to install anything.

## Getting in

Covered in [Ubuntu Setup](ubuntu-setup.md), and again in
[Week 3, Part C](labs/week-03.md#part-c-start-the-container). Short version:

```bash
cd ~
bash startDocker.sh        # start it once
bash connectToDocker.sh    # every additional terminal
```

Your prompt changes when you are inside.

!!! danger "The container is deleted every time you exit"
    It runs with `--rm`. Anything you install or create inside is gone when you
    type `exit` — `sudo apt install` included.

    Two directories survive, because they are shared with your own machine:

    ```
    ~/ENME480_mrc/src     <-->  ~/enme480_ws/src
    ~/ENME480_mrc/config  <-->  ~/enme480_ws/config
    ```

    Put your work there and nowhere else.

## Checking ROS 2 works

Inside the container:

```bash
ros2 topic list
```

You should see `/parameter_events` and `/rosout`. That is ROS 2 running.

For a fuller check, in two terminals:

```bash
ros2 run demo_nodes_cpp talker      # terminal 1
ros2 run demo_nodes_cpp listener    # terminal 2
```

The listener should print what the talker sends.

To check graphical programs can reach your screen, which is the thing most
likely to be broken:

```bash
rqt
```

If no window appears, see the troubleshooting section in
[Ubuntu Setup](ubuntu-setup.md).

## Your workspace

**It already exists, at `~/enme480_ws`.** Do not create one.

This matters: the official ROS 2 tutorials have you build a workspace at
`~/ros2_ws`, which is outside the shared folders above. Work there and it is
destroyed when the container shuts down, with no warning.

```bash
cd ~/enme480_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` links to your Python files rather than copying them, so
editing a node does not need a rebuild. You only need `colcon build` again after
adding a file or changing `setup.py`.

### Sourcing

Handled for you. The image writes this into `~/.bashrc`, so every new terminal
sources ROS 2 and your workspace automatically.

One exception worth remembering: a terminal opened **before** a `colcon build`
does not know about anything built since. Either open a new terminal or run:

```bash
source ~/enme480_ws/install/setup.bash
```

`Package not found` immediately after a successful build is almost always this.

## Command reference

### Looking at a running system

```bash
ros2 node list                  # what is running
ros2 topic list                 # what topics exist
ros2 topic echo /topic_name     # watch messages go past
ros2 topic info /topic_name     # message type, publisher and subscriber counts
ros2 topic hz /topic_name       # publishing rate
ros2 interface show <msg type>  # the fields inside a message
```

`ros2 topic info` is the one to reach for when two nodes will not talk. A
mismatched topic name throws no error — the nodes just never find each other.
If a count is 0 when you expect 1, you have a name or type mismatch, or
something is not running.

### Running things

```bash
ros2 run <package> <executable>
ros2 launch <package> <launch file>
ros2 pkg executables <package>      # what a package can run
```

### Parameters and services

```bash
ros2 param list
ros2 param get /node_name parameter_name
ros2 service list
ros2 service call /service_name <service type> "data"
```

### Graphical tools

| Tool | What it is for |
|---|---|
| `rqt` | Plugin GUI. Node Graph and Topic Monitor are the useful ones |
| `rviz2` | 3D visualisation of robot state and sensor data |

## Learning ROS 2

The official beginner tutorials are mirrored on this site, so you can read them
without being sent to a guide whose paths do not match our setup:

- [ROS 2 Tutorials](ros2-tutorials/index.md)

Start with
[Writing a simple publisher and subscriber](ros2-tutorials/writing-a-publisher-subscriber.md).
It explains every line of a talker node, which is the pattern nearly everything
in this course is built from.

Read them for the concepts. Ignore the parts that create a workspace or a
package — you already have both.

## Getting help

- **Official docs**: [docs.ros.org/en/humble](https://docs.ros.org/en/humble/)
- **ROS Answers**: [answers.ros.org](https://answers.ros.org/)
- **Piazza**, office hours, and lab sessions for anything course-specific

## Next

1. [Dev Environment](dev-environment.md) — VS Code against the container
2. [Week 3 Lab](labs/week-03.md) — your first ROS 2 nodes
