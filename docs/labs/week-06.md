# Week 06 — Forward Kinematics Lab 1.2

## Objectives

- Write a publisher for moving the robot in Gazebo
- Derive the Forward Kinematics for the UR3e
- Compare against the values obtained from last week's lab 

## Useful Files

- UR3e Dimensions - 
- 

## Procedure

### Step 1: Pull the latest version of the Repo

The repository and docker files has been updated to include the updated simulation tools so you'll need to build the docker environment again

Before doing that take a backup of your current `/src` folder so that you don't accidentally lose access to your previous work.

```bash
cd
mkdir -p backup/week5
cp -r ~/ENME480_mrc/src/ ~/backup/week5
```

Next, we pull the latest version of the repository

```bash
cd ~/ENME480_mrc
git checkout .
git pull
```

**For MacOS users**, change Line no. 4 in the docker file `humble-enme480_ur3e.Dockerfile` at `~/ENME480_mrc/docker`

```
# BEFORE
FROM osrf/ros:humble-desktop AS humble-mod_desktop

# AFTER
FROM arm64v8/ros:humble AS humble-mod_desktop
```

**Do not do this on anything other than a MAC!** MACs require code that has been compiled in a special way in order to work and this code does not work on other computers!


### Step 2: Build and run docker

**For Everyone**, run

```bash
cd ~/ENME480_mrc/docker/
userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-compose.yml build
```

Create the `startDocker.sh` and `connectToDocker.sh` scripts again if you haven't yet

For people **not** using the Nvidia container, run:
```bash
cd

echo -e "#"'!'"/bin/bash\nexport userid=$(id -u) groupid=$(id -g)\ncd ~/ENME480_mrc/docker\ndocker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker" > startDocker.sh

echo -e "#"'!'"/bin/bash\ncontainer="'$(docker ps | grep docker-enme480_ur3e-docker-run | cut -b 1-12)'"\necho Found running container "'$container'". Connecting...\ndocker exec -ti "'$container'" bash" > connectToDocker.sh
```

For people who **are** using the Nvidia container, run:
```bash
cd 

echo -e "#"'!'"/bin/bash\nexport userid=$(id -u) groupid=$(id -g)\ncd ~/ENME480_mrc/docker\ndocker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker" > startDocker.sh

echo -e "#"'!'"/bin/bash\ncontainer="'$(docker ps | grep docker-enme480_ur3e-docker-run | cut -b 1-12)'"\necho Found running container "'$container'". Connecting...\ndocker exec -ti "'$container'" bash" > connectToDocker.sh
```

### Step 3: Build the workspace

Now, we build the workspace for the simulation

```bash
cd ~/enme480_ws
colcon build
```

Once done, source it

```bash
cd ~/enme480_ws
source install/setup.bash
```

### Step 4: Launch the Simulation

Now we will test if the simulation environment is working

```bash
cd ~/enme480_ws
source install/setup.bash
ros2 launch enme480_sim enme480_ur3e_sim.launch.py
```

This should open up a Gazebo environment with the UR3e arm on a table with a vacuum grippper on it's end effector.

You can test if the robot is operational by running the followig command

```bash
ros2 run ur_robot_driver example_move
```