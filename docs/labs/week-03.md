---
title: Week 3 — ROS 2 Basics (Python)
icon: material/robot-industrial
description: Verify your Week 2 setup, then write a ROS 2 subscriber that keeps a running sum and a publisher that drives turtlesim in a circle.
---

# Week 3 · ROS 2 (Humble) with Python

This week you write your first two ROS 2 nodes.

The point of this lab is **not** to learn all of ROS 2. It is to get comfortable
with the loop you will use for the rest of the semester: a workspace, a package
inside it, Python files inside that, build, source, run. The ROS concepts you
need (publishers, subscribers, topics, messages) show up along the way.

We have given you the package already built and wired up. You fill in ten lines
across two files.

**Time:** about 2 hours. Parts A to D are setup and should take 30 minutes.

!!! tip "Reference reading"
    The official ROS 2 tutorials explain the ideas behind all of this in much
    more depth. They are mirrored on this site so you can read them alongside
    the lab: [ROS 2 Tutorials](../ros2-tutorials/index.md).

    The one worth reading before Part F is
    [Writing a simple publisher and subscriber](../ros2-tutorials/writing-a-publisher-subscriber.md),
    which walks through every line of a talker node.

!!! note "Where this picks up from"
    You should have finished the [Ubuntu Setup](../ubuntu-setup.md) guide in
    Week 2, up to and including the "Tests for Week 2" section. Part A checks
    that. If any of it fails, fix that before going further, and ask a TA if you
    are stuck.


## Part A — Check your Week 2 setup

Run these on your own machine, **outside** Docker.

| # | Run this | You should see |
|---|----------|----------------|
| 1 | `ls ~/ENME480_mrc` | folders including `docker`, `src`, `config` |
| 2 | `docker --version` | a version number, not "command not found" |
| 3 | `docker images` | a row whose name contains `enme480_ur3e` |

If check 1 fails, you never cloned the course repo. If check 2 fails, Docker is
not installed. If check 3 fails, you never built the image. All three are in the
[Ubuntu Setup](../ubuntu-setup.md) guide under "ENME480 Docker Installation".

!!! tip "If any of those fail, or Week 2 threw errors you scrolled past"
    Run the full dependency check in
    [Ubuntu Setup, Step 4](../ubuntu-setup.md#step-4-check-your-install). It
    tests every package Week 2 was supposed to install and prints OK or FAIL for
    each one, and the Repair section right below it fixes the common failures.


## Part B — Update the container

The course image has been updated since Week 2 to include `rqt`, which you need
this week. Pull the changes and rebuild.

```bash
cd ~/ENME480_mrc
git pull
```

Then rebuild the image:

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

!!! note "This is much faster than Week 2"
    Docker reuses everything it already built. Only the changed step and the
    few steps after it are redone, so expect a few minutes, not the long wait
    you had the first time. Most of that is downloading the `rqt` packages.

Use the same one of these two tabs for the whole lab. If you did the NVIDIA
step in Week 2, you are on the NVIDIA tab. Otherwise you are on the standard
tab.


## Part C — Start the container

### C1. Make two shortcut scripts

Typing the full `docker compose` command every time gets old fast. These two
small scripts do it for you. Run this once, from your home folder:

=== "Standard (no NVIDIA GPU)"

    ```bash
    cd ~
    cat > startDocker.sh << 'EOF'
    #!/bin/bash
    export userid=$(id -u) groupid=$(id -g)
    cd ~/ENME480_mrc/docker
    docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
    EOF
    ```

=== "NVIDIA GPU"

    ```bash
    cd ~
    cat > startDocker.sh << 'EOF'
    #!/bin/bash
    export userid=$(id -u) groupid=$(id -g)
    cd ~/ENME480_mrc/docker
    docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
    EOF
    ```

Then, for everyone:

```bash
cd ~
cat > connectToDocker.sh << 'EOF'
#!/bin/bash
container=$(docker ps | grep docker-enme480_ur3e-docker-run | cut -b 1-12)
echo "Found running container $container. Connecting..."
docker exec -ti "$container" bash
EOF
```

`cat > file << 'EOF'` writes everything up to the closing `EOF` into that file.
Open `startDocker.sh` in your editor and read it. It is just the commands you
already ran in Week 2, saved so you do not have to retype them.

Check both files landed where you expect:

```bash
ls ~/startDocker.sh ~/connectToDocker.sh
```

### C2. Start it

```bash
cd ~
bash startDocker.sh
```

Your prompt changes to show you are inside the container.

!!! warning "Start it once, connect many times"
    You will need several terminals this week. Run `startDocker.sh` **once**.
    For every additional terminal, open a new one on your own machine and run:

    ```bash
    cd ~
    bash connectToDocker.sh
    ```

    Running `startDocker.sh` more than once creates separate containers that
    cannot see each other's topics, which produces confusing failures later. If
    you do it by accident, type `exit` in the extra one.

### C3. Pre-flight checks

Run these **inside** the container. All four must pass before you continue.

| # | Run this | You should see |
|---|----------|----------------|
| 1 | `ros2 topic list` | `/parameter_events` and `/rosout` |
| 2 | `echo $AMENT_PREFIX_PATH` | a path containing `enme480_ws` |
| 3 | `ros2 run turtlesim turtlesim_node` | a blue window with a turtle |
| 4 | `rqt` | the rqt window opens |

Close the turtlesim window and press `Ctrl+C` in that terminal when you are
done with check 3. Same for `rqt`.

Check 2 confirms your workspace is being sourced automatically. Checks 3 and 4
confirm that graphical programs inside the container can reach your screen,
which is the thing most likely to be broken. If either window fails to appear,
see [Troubleshooting](#troubleshooting) at the bottom, and ask a TA.


## Part D — Get the Week 3 package

We have made a package for you with the structure, dependencies and entry points
already set up. Clone it into the course repo's `src` folder.

Run this **on your own machine, outside the container**, in a new terminal:

```bash
cd ~/ENME480_mrc/src
git clone https://github.com/ENME480/enme480_week3.git
```

Then, **inside the container**, build and source the workspace:

```bash
cd ~/enme480_ws
colcon build --symlink-install
source install/setup.bash
```

Check it worked:

```bash
ros2 pkg executables enme480_week3
```

You should get three lines: `talker`, `listener_sum` and `turtle_circle`.

!!! note "Why the folders are different but the same"
    `~/ENME480_mrc/src` on your machine and `~/enme480_ws/src` inside the
    container are the same folder. Anything you put in one appears in the other.
    **This is the only place your work survives.** The container is deleted every
    time you exit it, so files you create anywhere else are gone.

    This also means you can edit these files in VS Code on your own machine and
    run them in the container, without copying anything back and forth.

!!! note "`--symlink-install`"
    That flag means the build links to your Python files instead of copying
    them. You can edit a node and run it again without rebuilding. You only need
    to run `colcon build` again if you add a new file or change `setup.py`.


## Part E — Read and run the talker

Open `~/ENME480_mrc/src/enme480_week3/enme480_week3/talker.py` in your editor
and read it. It is about 40 lines and every one is commented. This is the
file you will copy patterns from for the rest of the lab.

Run it:

```bash
ros2 run enme480_week3 talker
```

It prints a counter, once a second.

Leave it running, and in a **second** terminal (remember: `bash connectToDocker.sh`)
look at what it is doing:

```bash
ros2 topic list
ros2 topic echo /numbers
ros2 topic info /numbers
```

`ros2 topic list` shows every topic that exists right now. `echo` prints the
messages going across one. `info` tells you the message type and how many nodes
are publishing and subscribing to it.

**Checkpoint:** you can see `/numbers` in the topic list, and `echo` prints a
number roughly once a second.

**Read more:** [Understanding topics](../ros2-tutorials/understanding-topics.md)
covers `list`, `echo`, `info` and `pub` properly, and shows how to draw the same
picture with `rqt_graph`.


## Part F — Finish `listener_sum.py`

Open `enme480_week3/listener_sum.py`. There are **six TODOs**.

This node subscribes to `/numbers`, adds up everything it hears, and publishes
the running total on `/sum_topic`. So it is a subscriber and a publisher at the
same time, which is the one genuinely new idea this week.

The pattern is:

1. In `__init__`, create the subscription, create the publisher, set the total to 0.
2. ROS calls `number_received()` for you, once per incoming message.
3. Inside that function, update the total and publish it.

Everything you need is either in the TODO comments or in `talker.py`.

**Read more:**
[Writing a simple publisher and subscriber](../ros2-tutorials/writing-a-publisher-subscriber.md)
explains the talker and listener line by line. Note that it builds both as
separate nodes; yours has to do both jobs in one.

Test it with the talker running in one terminal:

```bash
ros2 run enme480_week3 listener_sum
```

and in a third terminal:

```bash
ros2 topic echo /sum_topic
```

**Checkpoint:** the totals climb 0, 1, 3, 6, 10, 15 and so on. If the talker has
been running a while before you start the listener, your sum starts from
whatever number it is up to, which is fine and expected.


## Part G — Finish `turtle_circle.py`

Open `enme480_week3/turtle_circle.py`. There are **four TODOs**.

Start the simulator in one terminal:

```bash
ros2 run turtlesim turtlesim_node
```

Before writing anything, find out what the turtle listens to. In another
terminal:

```bash
ros2 topic list
ros2 interface show geometry_msgs/msg/Twist
```

The first shows you which topic carries velocity commands. The second shows you
the six numbers inside a `Twist` message. Only two of them matter for a turtle
on a flat screen.

Then fill in the TODOs and run it:

```bash
ros2 run enme480_week3 turtle_circle
```

**Checkpoint:** the turtle draws a clear circle and stays on screen. If it goes
straight, your turning rate is zero. If it spins on the spot, your forward speed
is zero. If it runs into a wall, your circle is too big: the radius is forward
speed divided by turning rate.

Finally, open `rqt` and have a look at what you built:

```bash
rqt
```

From the menu, open **Plugins → Introspection → Node Graph**. It draws your
nodes as boxes and your topics as arrows between them. Try **Plugins → Topics →
Topic Monitor** as well.


## Further reading

You did not have to create the workspace or the package this week, because we
gave them to you. You will want to know how that is done, and these are the
official ROS 2 tutorials that explain it. Mirrored here, with a link to the
original on each page.

| Tutorial | Why you would read it |
|----------|----------------------|
| [Writing a simple publisher and subscriber](../ros2-tutorials/writing-a-publisher-subscriber.md) | Every line of a talker and a listener, explained |
| [Understanding topics](../ros2-tutorials/understanding-topics.md) | The CLI tools you used in Parts E to G |
| [Creating a package](../ros2-tutorials/creating-a-package.md) | What `package.xml` and `setup.py` actually do |
| [Creating a workspace](../ros2-tutorials/creating-a-workspace.md) | Overlays, underlays, and why sourcing matters |

!!! warning "One difference from our setup"
    Those tutorials create a workspace at `~/ros2_ws`. **Do not follow that
    part.** Your workspace is `~/enme480_ws`, and it is the only location that
    survives the container shutting down. Read them for the concepts, not for
    the paths.


## Deliverables

Submit **one PDF** with four screenshots, plus your two Python files.

Screenshots:

1. Your `listener_sum` terminal, showing the running total climbing.
2. `ros2 topic echo /sum_topic`, a few lines is plenty.
3. The turtlesim window with a clear circular path drawn.
4. rqt showing the **Node Graph** with your nodes and topics in it.

One image can cover more than one of these if the windows are side by side.

Files, submitted separately or as an appendix:

- `listener_sum.py`
- `turtle_circle.py`

You do not need to submit `talker.py`, since you did not change it.


## Troubleshooting

**`Package 'enme480_week3' not found`**

The terminal you are in was opened before you built the package. Open a new one,
or run `source ~/enme480_ws/install/setup.bash` again in the one you have. New
terminals source the workspace for you automatically; a terminal that was
already open does not know about anything built since.

**No window appears for turtlesim or rqt**

The container cannot reach your display. On WSL, this is usually the XAuthority
problem covered in the [Ubuntu Setup](../ubuntu-setup.md) troubleshooting
section. Check that you started the container with the right compose file for
your machine (NVIDIA or standard). If it still fails, ask a TA.

**A node runs but nothing happens**

Topic names and message types have to match exactly on both ends, and a mismatch
is silent. Nothing crashes, the two nodes just never find each other. Use:

```bash
ros2 topic list
ros2 topic info /your_topic_name
```

`info` tells you the publisher and subscriber counts. If either is 0 when you
expect 1, you have a name or type mismatch, or one of the nodes is not running.

**Two nodes cannot see each other**

You probably have more than one container running. Check with `docker ps` on
your own machine. If there is more than one row, `exit` out of the extras and
use `bash connectToDocker.sh` for additional terminals instead of
`startDocker.sh`.

**My edits do not seem to do anything**

Check you are editing the file under `~/ENME480_mrc/src/enme480_week3/`, and
that you saved it. If you added a new file or changed `setup.py`, you do need to
`colcon build` again.

**`git status` in `ENME480_mrc` shows `src/enme480_week3` as untracked**

That is normal. The Week 3 package is its own repository living inside the
course one. Leave it alone.
