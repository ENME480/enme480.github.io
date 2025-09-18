---
title: Week 3 — ROS 2 Basics (Python)
icon: material/robot-industrial
description: Create a ROS 2 workspace and package; implement a talker/listener; drive turtlesim; inspect with CLI and rqt.
---

# Week 3 · ROS 2 (Humble) with Python

This week you’ll learn how ROS 2 is organized and practice the core ideas you’ll use all semester: **workspaces**, **packages**, **publish/subscribe**, and a tiny sim (**turtlesim**). The steps below point you to the **official Humble tutorials**—follow them carefully.

---

## Part A - Setup

First, we'll show you how to make shortcut commands to launch your Docker image. The commands you need to run will vary depending on wether or not you are using the Nvidia container.

For people **not** using the Nvidia container, run:
```bash
echo -e "#"'!'"/bin/bash\nexport userid=$(id -u) groupid=$(id -g)\ncd ~/ENME480_mrc/docker\ndocker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker" > startDocker.sh

echo -e "#"'!'"/bin/bash\ncontainer="'$(docker ps | grep docker-enme480_ur3e-docker-run | cut -b 1-12)'"\necho Found running container "'$container'". Connecting...\ndocker exec -ti "'$container'" bash" > connectToDocker.sh
```

For people who **are** using the Nvidia container, run:
```bash
echo -e "#"'!'"/bin/bash\nexport userid=$(id -u) groupid=$(id -g)\ncd ~/ENME480_mrc/docker\ndocker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker" > startDocker.sh

echo -e "#"'!'"/bin/bash\ncontainer="'$(docker ps | grep docker-enme480_ur3e-docker-run | cut -b 1-12)'"\necho Found running container "'$container'". Connecting...\ndocker exec -ti "'$container'" bash" > connectToDocker.sh
```

These commands will create two bash files (essentially just lists of other commands) that will allow you to launch the Docker container with:
```bash
bash startDocker.sh
```

And connect to it from another terminal with:
```bash
bash connectToDocker.sh
```
Provided you are in the folder where these files are.

**IMPORTANT:** This command will create links between the following folders (outside the docker --> inside the docker)

```
ENME480_mrc/src --> enme480_ws/src
ENME480_mrc/config --> enme480_ws/config
```
This is where you should place any files you want to keep when the docker shuts down (i.e. assignment code). Any changes made to files in these folders in the docker will be reflected outside the docker and vice versa.
(Credit to Benjamin Ruby for the original version of the script)

---

## 🛫 Part B — Pre-flight check (5–10 min)

<!-- 1) Create a folder for this course (e.g., `~/enme480_ws`) to keep things tidy. -->
1) Using the commands from above, open your Docker image contianing ROS
2) Open a new terminal and verify ROS 2 is available (e.g., `ros2 --version` or `ros2 --help`).  
```bash
ros2 topic list
```
It will show you a list of topics, most likely `/rosout` and `/parameter_events`

3) Try opening up Gazebo
```bash
ign gazebo
```
It should open up a window with examples to different test environments

4) Try opening up `rqt`
```bash
rqt
```
If `rqt` does not open up anything or throws an error, install any required extenstions
```bash
sudo apt install ros-humble-rqt*
```
or try reopening it with 
```bash
rqt --force-discover
```

### Troubleshooting

During any of these steps if your display doesn't open up, follow the follwoing steps

<!--4) Create a symlink into the docker container by opening a new terminal (while leaving the container open) and running:

```bash
docker exec <HIT TAB> ln -s ~/<NAME OF THE FOLDER YOU JUST MADE> ~/<NAME YOU WANT THE FOLDER TO HAVE INSIDE DOCKER> 
```
This will cause the folder you just created to appear inside the docker image, letting you work inside of it without deleting your work when the container closes. This is called a *symbolic link*. We will provide instrucitons on how to make this permanent soon, but for the time being you will need to rerun this command each time you restart the container.-->


**Checkpoint B (no submission yet):** You can run the above commands without any errors.

---

## 📦 Part C — Workspace & package (setup only)

Follow the **official Humble tutorials** step-by-step (do not copy solution code from elsewhere):

1) **Create a workspace:** Use the **Create a workspace** guide and build once so the structure is valid.  
   ↪ Guide: [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

2) **Create a Python package:** Inside your `src/`, make a new package for this week (any sensible name, e.g., `week3`).  
   ↪ Guide: [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

**Checkpoint C:** Your workspace builds with `colcon` and your package appears in the build output.

---

## 📡 Part D — Talker / Listener

Use the **publisher/subscriber (Python)** tutorial as your primary reference:

- Tutorial: [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

**Task C1 — Publisher (“talker”)**  
Create a node that **publishes numbers at a steady rate** on a topic you choose (e.g., `/numbers`).  
*(Use the tutorial to recall how to create a publisher node; adapt it to publish numbers rather than strings.)*

**Task C2 — Subscriber (“listener”)**  
Create a node that **subscribes** to your numbers topic and **maintains a cumulative sum**. After each new message arrives, it should **publish the running sum** on a **new topic** (e.g., `/sum_topic`).  
*(Re-use the subscriber pattern from the tutorial; add your own sum logic and a second publisher.)*

**Task C3 — Inspect with CLI**  
Use the **Understanding topics** tutorial to: **list topics**, **echo** your sum topic, and **show** topic info (type, publishers/subscribers).  
↪ [docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

> **Hints (conceptual, not solutions):**
> - Topic names must **match exactly**; message types must be consistent.  
> - If a terminal shows “no publisher/subscriber”, confirm both nodes are running and your workspace is sourced.  
> - Keep node/topic names short and meaningful.

**Checkpoint D — Screenshots to capture:**  
- Talker output (brief).  
- Listener output showing a **running sum**.  
- `ros2 topic list` and a short `ros2 topic echo` of your sum topic.

---

## 🐢 Part E — Turtlesim (drive in a circle)

Read the **turtlesim, ros2, and rqt** tutorial first:  
[docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

**Task D1 — Launch turtlesim**  
Start the turtlesim simulator in one terminal (see tutorial).

**Task D2 — Velocity publisher**  
In your package, create a node that **publishes velocity commands** to turtlesim so the turtle moves in a **circle** (non-zero linear \(x\) and angular \(z\)).  
*(You’ll find the correct topic name in the tutorial; use the CLI to explore message fields.)*

**Task D3 — Observe & Inspect**  
Use `ros2 topic list` / `ros2 topic echo` to confirm motion and pose updates; open **rqt** and view the **Node Graph** and **Topic Monitor**.  
- rqt info: [docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html)

**Checkpoint E — Screenshots to capture:**  
- Turtlesim window with a **clear circular path**.  
- Your velocity publisher terminal (brief output).  
- `ros2 topic list` and a short `ros2 topic echo` of the pose topic.  
- rqt with **Node Graph** / **Topic Monitor** visible.

---

## 🔎 What we’re assessing

- You can follow **official Humble docs** and adapt examples to a new task.  
- Your nodes **publish/subscribe** correctly and use appropriate **topic names** and **message types**.  
- You can **inspect** systems via CLI and **rqt** and explain what you see.

---

## 📤 Deliverables (single PDF upload)

Include **concise** screenshots (one image may show multiple windows):

- **Talker/Listener:** talker output; listener output showing **running sum**; `ros2 topic list`; brief `ros2 topic echo` of your sum topic.  
- **Turtlesim:** turtlesim showing a **circle**; velocity publisher terminal; `ros2 topic list`; brief `ros2 topic echo` of the pose topic; rqt with Node Graph/Topic Monitor visible.  
- A terminal view of your **package tree** (folder/files) helps grading.

Also submit the **three Python files** you created this week as separate attachments or in the PDF appendix (clearly named).

