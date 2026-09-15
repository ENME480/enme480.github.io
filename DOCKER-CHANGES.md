# Dockerfile change needed before Week 3 goes live

Applies to `MarylandRoboticsCenter/ENME480_mrc`, file
`docker/humble-enme480_ur3e.Dockerfile`.

Nothing in this repo touches that one. Apply this by hand, then delete this file.

## What is missing today

The image installs `ros-humble-ros-gz-*`, `ros-humble-usb-cam` and
`ros-humble-moveit-*` (lines 63-67), but **not rqt**. That is why both
`week-03.md` and `week-06.md` currently tell students to run
`sudo apt install ros-humble-rqt*` by hand inside the container.

The container runs with `--rm`, so that install is thrown away the moment they
type `exit`. They redo it every single lab session.

`ros-humble-tf-transformations` has the same problem in Week 6.

## What to add

Insert this block **after line 133** (the `pydantic==1.10.9` line, at the end of
the `pip3 install` in the `humble-enme480_ur3e_ws` stage) and **before** the
`# Set up UR3e workspace` comment on line 135:

```dockerfile
# ENME480 lab tooling: rqt plugins (Week 3), tf helpers (Week 6)
RUN sudo apt-get update && sudo apt-get install -y \
    "ros-humble-rqt*" \
    ros-humble-turtlesim \
    ros-humble-tf-transformations && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
```

So the result reads:

```dockerfile
# adding missing python modules
RUN pip3 install keyboard \
    pydantic==1.10.9

# ENME480 lab tooling: rqt plugins (Week 3), tf helpers (Week 6)
RUN sudo apt-get update && sudo apt-get install -y \
    "ros-humble-rqt*" \
    ros-humble-turtlesim \
    ros-humble-tf-transformations && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Set up UR3e workspace
RUN source /opt/ros/humble/setup.bash && \
	mkdir -p $HOME/${WS_DIR}/src && \
    ...
```

## Why it goes after line 133 and not in the block at lines 63-67

Docker throws away every cached layer below the one you edit.

- **After line 133:** only the three cheap layers below it rebuild (an empty
  `colcon build` and three `echo` calls into `.bashrc`). Rebuild is the apt
  download and little else, **roughly 3-8 minutes**.
- **Inside lines 63-67:** invalidates lines 116-124, which clone and
  `colcon build` the Universal Robots drivers from source. That is the **full
  20-40 minute** first-build experience again, for every student.

The Week 3 page tells students the rebuild is quick. That is only true if the
block goes in at the bottom.

## Notes

`ros-humble-turtlesim` is already pulled in by `osrf/ros:humble-desktop`, so on a
normal build it is a no-op. It is listed explicitly so the lab does not silently
depend on what the desktop metapackage happens to include.

## What does NOT need changing

Sourcing is already handled. Lines 143-149 write these into `.bashrc`:

```dockerfile
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc && \
    ...
    echo "source $HOME/${WS_DIR}/install/setup.bash" >> $HOME/.bashrc && \
```

So every new shell already sources `~/enme480_ws/install/setup.bash`. The lab
only needs to tell students that a terminal opened *before* a `colcon build`
will not see the new package until they re-source or open a new terminal.

## Order of operations

1. Apply the block above to `ENME480_mrc` and push.
2. Push `enme480_week3` to the `ENME480` org.
3. Merge the `week3-streamline` branch here.

If 3 happens before 1, students run `git pull` and rebuild for no reason, and
`rqt` still will not start.
