---
title: Week 4 — Gazebo & Python
icon: material/robot-industrial
description: Studio 4.1 brings up the UR3e in Gazebo and moves it. Studio 4.2 builds rotation matrices in Python.
---

# Week 4 — Gazebo & Python

Two studios this week.

| | |
|---|---|
| **Studio 4.1** | Confirm Gazebo works, bring up the UR3e, move it |
| **Studio 4.2** | Build rotation matrices in Python |

4.1 needs the container. 4.2 is a standalone script and does not touch ROS at
all, so you can do it while the simulator is loading.

The commands you send this week are the **same ones that drive the real arm** in
Week 5. That is the point of the simulator: if it works here, it should work on
the robot.

**Time:** about 30 minutes for 4.1 if your Week 3 setup still works, plus 30 for 4.2.

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


## Studio 4.1 — Gazebo & the UR3e

### Part A — Start the container and check Gazebo

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


### Part B — Add the helper package

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


### Part C — Bring up the UR3e

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

**Pane 1 — the simulator.** Gazebo opens with the UR3e in an empty world.

```bash
ros2 launch enme480_gazebo enme480_ur3e_empty.launch.py launch_rviz:=true
```

!!! warning "`launch_rviz:=true` is not optional here"
    That argument defaults to **false**, so without it you get Gazebo and no
    RViz — and the deliverables for this lab need both. If you have already
    launched without it, stop the pane with `Ctrl+C` and run it again with the
    argument.

RViz does not appear immediately. It is held back until the joint state
broadcaster is up, so expect a few seconds of Gazebo on its own first. That is
deliberate: starting RViz earlier would show a robot with no joint data and
every link stacked at the origin.

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

### What RViz is showing you

Gazebo is the *physics simulation* — it is the robot. RViz is a *viewer* for
what ROS believes is going on: the links, where TF says each frame is, and the
joint positions coming back on `/joint_states`.

They can disagree, and that disagreement is the useful part. If the arm moves in
Gazebo but not in RViz, the robot moved and ROS was not told, which usually
means a broken publisher rather than a broken robot.

In the left-hand Displays panel you can toggle:

| Display | What it shows |
|---------|---------------|
| **RobotModel** | the arm's links, drawn from the URDF |
| **TF** | a set of axes at every frame, including each joint and the tool |
| **Grid** | the ground plane, for scale |

Turning **TF** on is worth doing now. Those axes are the frames you will be
assigning DH parameters to in Week 6, and it is much easier to reason about them
once you have watched them move.

### Opening RViz separately

If you close RViz, or want it in its own pane, you do not need to restart the
simulator:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix enme480_description)/share/enme480_description/rviz/view_robot.rviz
```

Without `-d` and that config file you get an empty RViz and have to add the
displays by hand.


### Part D — Move it

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
ros2 topic echo /ur3e/position
```

`/joint_states` is what the joints are actually doing. `/ur3e/position` is the
computed end effector position.

### Run these three poses

```text
[0, -0.758, 0, 0.758, -1.571, 1.048]
[-0.524, -1.048, 1.396, -0.175, -1.571, -0.524]
[0.524, -1.222, 1.396, -0.175, -1.571, 0.175]
```

**Checkpoint D:** the arm visibly moves to a different configuration for each, and `/joint_states` reports angles close to what you sent. Note that the angles reported by ROS2 may not be the same as what you requested. This is fine, we will discuss why this is the case once we get to Kinematics labs. 


## Studio 4.2 — Rotation matrices in Python

No ROS, no container needed. This one is about the maths you will use for the
rest of the semester: a rotation matrix turns a vector expressed in one frame
into the same vector expressed in another, and chaining two of them applies both
rotations in order.

### Get the script

```bash
curl -fsSLO https://enme480.github.io/assets/studio_4_2.py
```

Or [download it here](../assets/studio_4_2.py). It runs as-is — it just does
nothing useful until you fill it in.

You need Python 3 and NumPy. Both are already in the container. On your own
machine, `pip install numpy` if `import numpy` fails.

### What to write

**1. Finish `GetRotationMatrix(phi)`** so it returns a rotation about the z axis:

$$
R(\phi) =
\begin{bmatrix}
\cos\phi & -\sin\phi & 0 \\
\sin\phi & \cos\phi & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

!!! warning "R has to be 3×3"
    The stub starts with `R = np.zeros(3)`, which is a flat array of three
    zeros, not a matrix. Replace it. `np.zeros((3, 3))` — note the inner
    brackets — or build the array directly.

**2. Inside `Test()`, define the vector:**

$$
v_1 =
\begin{bmatrix}
1 \\
0.6 \\
0.8
\end{bmatrix}
$$

**3. Compute** $$v_2 = R(\phi_2)\,R(\phi_1)\,v_1$$ with `np.matmul()`, using the
`phi1` and `phi2` already defined in the script, and print `v2`.

### Run it

```bash
python studio_4_2.py
```

!!! tip "Two sanity checks"
    Rotation matrices preserve length, so `np.linalg.norm(v2)` must equal
    `np.linalg.norm(v1)`. And because both rotations are about z, the third
    component of `v2` should still be `0.8`.

    If either fails, your matrix is wrong — most likely a sign on one of the
    `sin` terms, or the multiplication order reversed.

Order matters: $R_2 R_1 v_1$ is not the same as $R_1 R_2 v_1$ in general,
though for two rotations about the *same* axis it happens to be. That is worth
noticing now, because it stops being true in Week 6.

## Deliverables

Submit **one PDF** to ELMS covering both studios.

**Studio 4.1**

1. Gazebo and RViz showing the UR3e in each of the **three poses** above, clearly labelled.
2. The reported joint angles for each pose, from `ros2 topic echo /joint_states`.

One screenshot per pose can cover both if the windows are side by side.

**Studio 4.2**

3. Your finished `studio_4_2.py`.
4. A screenshot of the terminal showing the value of `v2`.

Keep the function names and structure of the provided script. Check the rubric
on ELMS before submitting.


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
