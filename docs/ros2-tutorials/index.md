---
title: ROS 2 Tutorials
description: Mirrored copies of the official ROS 2 Humble beginner tutorials, for reference alongside the labs.
---

<p class="eyebrow">ENME480 · Wiki</p>

# ROS 2 Tutorials

These are copies of the official ROS 2 Humble tutorials, reproduced word for
word so you can read them without leaving the course site. They are the same
pages the ROS documentation serves, and each one links back to its original.

Read these to understand **why** things work the way they do. They are not the
assignment. What you have to do for credit is on the
[Week 3 lab page](../labs/week-03.md), and the steps there are written for our
Docker setup, which differs from the tutorials in a few places.

!!! warning "Where these differ from our setup"
    The tutorials tell you to create a workspace at `~/ros2_ws`. **Do not do
    that.** Your workspace already exists at `~/enme480_ws`, and it is the only
    place your work survives when the container shuts down. See
    [Week 3, Part D](../labs/week-03.md#part-d-get-the-week-3-package).

    They also have you create a package from scratch with `ros2 pkg create`. We
    have already done that for you in the
    [enme480_week3](https://github.com/ENME480/enme480_week3) package, so you
    can skip straight to writing node code.

## The tutorials

| Tutorial | What it covers |
|----------|----------------|
| [Creating a workspace](creating-a-workspace.md) | What a workspace is, overlays and underlays, sourcing, `colcon build` |
| [Creating your first ROS 2 package](creating-a-package.md) | What a package is, `package.xml`, `setup.py`, entry points |
| [Writing a simple publisher and subscriber](writing-a-publisher-subscriber.md) | The talker/listener pattern in Python, line by line |
| [Understanding topics](understanding-topics.md) | `ros2 topic list`, `echo`, `info`, `pub`, and `rqt_graph` |

If you only read one, read **Writing a simple publisher and subscriber**. It
explains every line of the talker, which is the pattern both of your Week 3
nodes are built from.

## Licence and attribution

The ROS 2 documentation is copyright Open Robotics and is licensed under
[Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/),
which permits redistribution with attribution. These pages are unmodified copies
retrieved on 15 September 2026, each carrying a link to its source.

Upstream may have changed since. For the current version, follow the source link
at the top of any of these pages, or start at
[docs.ros.org/en/humble](https://docs.ros.org/en/humble/).
