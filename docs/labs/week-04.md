---
title: Week 4 — Gazebo & the UR3e Simulator
icon: material/robot-industrial
description: Bring up the UR3e in Gazebo and command it to move, using the same topic interface as the real arm.
---

# Week 4 — Gazebo & the UR3e Simulator

The goal this week is narrow and practical: **confirm Gazebo works on your
machine, bring up the UR3e, and move it.** No kinematics yet, no code to write.
If you finish this lab you have a working simulator for the rest of the
semester.

The commands you send this week are the **same ones that drive the real arm** in
Week 5. That is the point of the simulator: if it works here, it should work on
the robot.

**Time:** about 30 minutes if your setup from Week 3 still works.

!!! note "Before you start"
    You need a working Week 2 setup. If anything there errored and you scrolled
    past it, check now:

    ```bash
    curl -fsSL https://enme480.github.io/assets/check_setup.sh | bash
    ```

    Every line must say `OK`. See
    [ROS 2 in the Container](../ros-setup.md) for a refresher on how the pieces
    fit together.


??? note "Only if something is out of date"

    Skip this unless a step below fails. Your setup from Week 3 should still
    work, and rebuilding the image takes 20 to 40 minutes for no reason.

    Come back here if `ign gazebo` will not start, or if a launch file in Part C
    reports `Package not found` even after a successful build.

    ```bash
    cd ~/ENME480_mrc
    git pull
    ```

    If `git pull` reports a conflict on `humble-enme480_ur3e.Dockerfile`, keep
    **your** version of line 4 — Mac and VM users edited it — and take the
    incoming changes everywhere else. Do not discard your local changes to get
    past it.

    Then rebuild:

    === "Standard (no NVIDIA GPU)"

        ```bash
        cd ~/ENME480_mrc/docker
        userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-compose.yml build
        ```

    === "NVIDIA GPU"

        ```bash
        cd ~/ENME480_mrc/docker
        userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
        ```


## Part A — Start the container and check Gazebo

```bash
cd ~
bash startDocker.sh
```

Inside the container:

```bash
ign gazebo
```

A window should open with a list of example worlds. That is the only check that
matters at this stage. Close it again.

If no window appears, the container cannot reach your display. That is the
single most common failure and it is not a Gazebo problem — see
[Gazebo in this course](../gazebo-setup.md) and the troubleshooting section of
[Ubuntu Setup](../ubuntu-setup.md).

**Checkpoint A:** `ign gazebo` opens a window.


## Part B — Add the helper package

One new package this week, which later labs also use. Run this **on your own
machine**, outside the container:

```bash
cd ~/ENME480_mrc/src
git clone https://github.com/ENME480/ur3e_enme480.git
```

If the folder already exists, you have it, move on.

Then **inside the container**, build and source:

```bash
cd ~/enme480_ws
colcon build --symlink-install
source install/setup.bash
```

**Checkpoint B:** `colcon build` finishes with no errors, and
`ros2 pkg list | grep enme480` shows `enme480_gazebo` and `ur3e_enme480`.


## Part C — Bring up the UR3e

You need four terminals. Use `tmux` rather than juggling four
`connectToDocker.sh` windows:

| Action | Keys |
|---|---|
| Start a session | `tmux` |
| Split horizontally | `Ctrl+A` then `b` |
| Split vertically | `Ctrl+A` then `v` |
| Move between panes | `Ctrl+A` then an arrow key |

Run one command per pane, **in this order**, waiting for each to settle before
starting the next.

**Pane 1 — the simulator.** Gazebo opens with the UR3e in an empty world, and
RViz opens alongside it.

```bash
ros2 launch enme480_gazebo enme480_ur3e_empty.launch.py
```

**Pane 2 — the MRC control layer.** This listens for your commands and drives
the arm's controllers.

```bash
ros2 launch ur3e_mrc_sim ur3e_enme480.launch.py
```

**Pane 3 — the ENME480 layer.** Publishes the end effector position you will use
from Week 6 onward.

```bash
ros2 launch ur3e_enme480 ur3e_sim_enme480.launch.py
```

**Checkpoint C:** Gazebo and RViz are both open showing a UR3e, and none of the
three panes is printing errors.


## Part D — Move it

In pane 4, send the arm a set of joint angles:

```bash
ros2 topic pub --once /ur3e/command ur3e_mrc_msgs/msg/CommandUR3e "destination: [0, -1.57, -1.57, 0, 0, 0]
v: 1.0
a: 1.0
io_0: false"
```

The arm should move in Gazebo. Breaking that message down:

| Field | Meaning |
|---|---|
| `destination` | six joint angles, **in radians**, one per joint |
| `v`, `a` | velocity and acceleration limits |
| `io_0` | laser pointer, only used on the real robot |

!!! danger "Radians, not degrees"
    `destination` is in **radians**. An entry like `90` is not 90 degrees, it is
    about 14 full rotations' worth of command. Sanity check every angle before
    you press enter. This habit matters in Week 5, when the same command moves a
    real arm near real people.

`--once` publishes a single message and exits, rather than repeating forever.

### Watch it report back

In a spare pane:

```bash
ros2 topic echo /joint_states
ros2 topic echo /ur3/position
```

`/joint_states` is what the joints are actually doing. `/ur3/position` is the
computed end effector position.

### Run these three poses

```text
[0, -0.758, 0, 0.758, -1.571, 1.048]
[-0.524, -1.048, 1.396, -0.175, -1.571, -0.524]
[0.524, -1.222, 1.396, -0.175, -1.571, 0.175]
```

**Checkpoint D:** the arm visibly moves to a different configuration for each,
and `/joint_states` reports angles close to what you sent.


## Deliverables

Submit **one PDF** with:

1. Gazebo and RViz showing the UR3e in each of the **three poses** above, clearly labelled.
2. The reported joint angles for each pose, from `ros2 topic echo /joint_states`.

One screenshot per pose can cover both if the windows are side by side.

No code this week.


## Troubleshooting

**`Package 'enme480_gazebo' not found`**

The terminal was opened before you built. Check the package is there, then
rebuild and source:

```bash
ls ~/ENME480_mrc/src
cd ~/enme480_ws && colcon build --symlink-install && source install/setup.bash
```

If `enme480_gazebo` is not listed by `ls` at all, your copy of the repo predates
it. That is the one case where you do need the update block near the top of this
page.

**Gazebo opens but the arm never appears**

Give it time; the first launch is slow while it loads meshes. If it is still
empty after a minute, read pane 1 from the top. The real error is usually the
first one, not the last.

**The arm does not move when you publish**

Check the command is reaching the control layer:

```bash
ros2 topic info /ur3e/command
```

You want at least one publisher and one subscriber. If the subscriber count is
0, pane 2 is not running or died on startup. A mismatched topic name produces no
error at all, so check the name character by character.

**Everything dies when I press Ctrl+C**

That is expected. Shutting down a launch file kills the nodes it started, and
some complain on the way out. Errors *during shutdown* are not a problem; errors
*during startup* are.
