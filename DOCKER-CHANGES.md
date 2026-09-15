# Dockerfile changes needed in ENME480_mrc

Applies to `MarylandRoboticsCenter/ENME480_mrc`, file
`docker/humble-enme480_ur3e.Dockerfile`.

Nothing in this repo touches that one. Apply these by hand, then delete this file.

There are **two** changes. Apply them together — see [Rebuild cost](#rebuild-cost)
for why doing them in one pass matters.

---

## Change 1 — the `moveit-*` glob breaks the build (lines 63-67)

### Symptom

```
Errors were encountered while processing:
 /tmp/apt-dpkg-install-.../1006-ros-humble-moveit-task-constructor-core_0.1.3-1jammy.20260910.041030_arm64.deb
E: Sub-process /usr/bin/dpkg returned an error code (1)
failed to solve: process "/bin/bash -c sudo apt-get update && sudo apt-get install -y ... ros-humble-moveit-* ..." did not complete successfully: exit code: 100
```

### Cause

`ros-humble-moveit-*` matches **82 packages** in the Humble arm64 index, not the
handful the line intends. Among them:

| Caught by the glob | Count | Installed size |
|---|---|---|
| Everything matching `ros-humble-moveit*` | 82 | 658 MB |
| …of which are `-dbgsym` debug symbols | 33 | 531 MB |
| …the whole `moveit-task-constructor-*` stack | 10 | — |

`moveit-task-constructor` is a separate research stack, not part of core MoveIt.
Nothing in this course uses it. It is only being installed because the glob is
wider than intended, and one of its packages fails to unpack.

`ros-humble-ros-gz-*` has the same problem on a smaller scale: it matches 10
packages, 4 of which are `-dbgsym`.

The build log the failure came from is truncated above the dpkg reason, so I
cannot say which specific conflict killed `task-constructor-core`. It does not
matter — the package should not be installed at all, and removing it from the
set removes the failure mode along with 531 MB of debug symbols.

### Fix

Replace the globs with the metapackages, which pull exactly the real packages
and none of the debug symbols.

Lines 62-67 currently read:

```dockerfile
# Install auxilary ROS packages, using new Gazebo
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros-gz-* \
    ros-humble-usb-cam \
    ros-humble-moveit-* && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
```

Change to:

```dockerfile
# Install auxilary ROS packages, using new Gazebo.
# Use the metapackages, not globs: ros-humble-moveit-* matches 82 packages
# including 33 debug-symbol packages and the unrelated moveit-task-constructor
# stack, which fails to unpack.
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-usb-cam \
    ros-humble-moveit && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
```

What the metapackages pull:

- `ros-humble-ros-gz` → `ros-gz-bridge`, `ros-gz-image`, `ros-gz-sim`, `ros-gz-sim-demos`
- `ros-humble-moveit` → `moveit-core`, `moveit-planners`, `moveit-plugins`, `moveit-ros`, `moveit-setup-assistant`

!!! note
    `ros-humble-moveit` may be droppable entirely. `ros-humble-ur` (installed at
    line 112) depends on `ros-humble-ur-moveit-config`, so MoveIt arrives
    transitively either way. Keeping it explicit is harmless and states intent,
    so the change above keeps it. Your call.

---

## Change 2 — rqt is not in the image

### Cause

Lines 63-67 install `ros-gz`, `usb-cam` and `moveit`, but **not rqt**. That is
why `week-03.md` and `week-06.md` both tell students to run
`sudo apt install ros-humble-rqt*` by hand inside the container.

The container runs with `--rm`, so that install is discarded the moment they
type `exit`. They redo it every lab session.
`ros-humble-tf-transformations` has the same problem in Week 6.

### Fix

Insert this block **after line 133** (the `pydantic==1.10.9` line) and **before**
the `# Set up UR3e workspace` comment on line 135:

```dockerfile
# ENME480 lab tooling: rqt plugins (Week 3), tf helpers (Week 6)
RUN sudo apt-get update && sudo apt-get install -y \
    "ros-humble-rqt*" \
    ros-humble-turtlesim \
    ros-humble-tf-transformations && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
```

`ros-humble-rqt*` is deliberately still a glob here — rqt ships as many small
plugin packages with no metapackage covering the set, and unlike moveit it has
no `-dbgsym` or unrelated stacks in its namespace. It is quoted so the shell
hands the pattern to apt rather than trying to expand it against the filesystem.

`ros-humble-turtlesim` is already pulled in by `osrf/ros:humble-desktop`, so on a
normal build it is a no-op. It is listed explicitly so the lab does not silently
depend on what the desktop metapackage happens to include.

---

## Rebuild cost

Docker discards every cached layer below the one you edit.

| Change | Position | Cost alone |
|---|---|---|
| Change 2 (rqt) | after line 133 | 3-8 min — only cheap layers below it |
| Change 1 (moveit) | lines 63-67 | 20-40 min — invalidates the from-source UR driver build at lines 116-124 |

Change 1 is unavoidable, so the full rebuild happens regardless. **Apply both
changes in the same pass** and pay that cost once rather than twice.

After both are in, a student's `git pull` plus rebuild is still the full 20-40
minutes, because their cache is invalidated at line 63 too. Week 3 Part B
currently tells them to expect a few minutes. Either warn students in advance,
or push a prebuilt image, or reword that note.

---

## What does NOT need changing

Sourcing is already handled. Lines 143-149 append to `.bashrc`:

```dockerfile
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc && \
    ...
    echo "source $HOME/${WS_DIR}/install/setup.bash" >> $HOME/.bashrc && \
```

So every new shell already sources `~/enme480_ws/install/setup.bash`. The lab
only needs to tell students that a terminal opened *before* a `colcon build`
will not see the new package until they re-source or open a new terminal.

---

## Order of operations

1. Apply both changes to `ENME480_mrc` and push.
2. Merge the `week3-streamline` branch here.

If 2 happens before 1, students pull and rebuild for nothing, and `rqt` still
will not start.

`enme480_week3` is already pushed: <https://github.com/ENME480/enme480_week3>
