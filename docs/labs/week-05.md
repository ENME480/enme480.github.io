# Week 5 - UR3e Intro & Operation

## Objectives

- Learn how to use the Pendant
- Interface UR3e with ROS packages on RAL machines
- Visualize ROS Processes

## Procedure

1.  First, you will set up the pendant

Press the power button, shown here:

![powerbutton](assets/robot_pics/power)

Once the robot is powered on, it will still be in a disarmed state. Press the button shown below to arm it. You should expect to hear a series of loud clicks; this is the brakes releasing.

![arming](assets/robot_pics/arm)

Next, you can try using free drive to control the robot. To do so, press the black button on the back of the teaching pendant. This will allow you to manually move each joint of the robot.

![freedrive](assets/robot_pics/freedrive)

While holding the freedrive button, you can also add constraints to the robots motion using the screen in the bottom right. Try adding some and seeing how they restrict motion.

![constraints](assets/robot_pics/constraints) 

The teaching pendant also has an E-Stop button on its face. *If the robot ever moves in a way you don't expect, E-Stop it.* A stopped robot is better than a hurt classmate or a broken robot. Try pressing it now. In order to release it, you'll need to spin the knob to pop it back up and then re-release the brakes.

![estop](assets/robot_pics/estop)

For todays lab, you'll be using the teaching pendant to control the position of each of the robots joints. To do this, go to the move screen. You'll see a list of each joint and it's current encoder angle, as well as a set of controls you can use to move the robot in a cartesian frame. Try the different controls out.

![move](assets/robot_pics/move)

Note that by convention we will call movements where you directly set the angle of the joint *Forward Kinematics* (which is what you are doing in class right now and what this lab is about), while motions where you drectly control the position and orientation of the end effector will be called *Inverse Kinematics*.

## 2. DH Parameters of UR3e

Link for UR3e specifications: https://www.universal-robots.com/media/1807464/ur3e-rgb-fact-sheet-landscape-a4.pdf

The PDF for UR3 dimensions is included in the folder for ```Week 6``` and the zero configuration (all joint angles are 0) for the robot looks like [given in this image](images/UR3e_Zero_angle_Config.png)

You need to create a DH-table for the robot and annotate the given PDF to show the frames and axes used. The unknowns here will be the joint angles. Include the base plate in your calculations as well. Note that our given zero frame is not consistent with the zero frame given by Universal Robotics.

<!--
## 3. Creating a Publisher script to move the robot

(NEW) Bridge packages for custom topics between ur_driver and ENME480 labs

- Clone the following repositories into your workspace

```bash
git clone https://github.com/MarylandRoboticsCenter/ur3e_mrc.git
git clone https://github.com/ENME480/ur3e_enme480.git
```
- Build and source your workspace.


We have a predefined custom message for obtaining position and sending commands:

CommandUR3e.msg 
```
float64[] destination
float64 v
float64 a
bool io_0
```
(destination is the set of joint angles `[theta1 theta2 theta3 theta4 theta5 theta6]`)

PositionUR3e.msg
```
float64[] position
bool is_ready
```

(position is the set of 6DoF pose of the end effector `[x y z roll pitch yaw]`)

Now run the following command:
```bash
ros2 launch ur3e_enme480 ur3e_sim_enme480.launch.py
```

You should be able to see the topics `/ur3/position` and `/ur3/command`. Refer to this [link](https://github.com/ENME480/ur3e_enme480/tree/main) for details of the package and its usage.

~~Using the topic ```/joint_trajectory_controller/joint_trajectory``` and the message type ```JointTrajectory``` and ```JointTrajectoryPoint``` from ```trajectory_msgs```, create a publisher to move the robot to desired joint angles. Keep in mind that the angles given to th robot sould be in radians but we want to give the input in degrees so ensure that you have converted that.~~

Using the topic ```/ur3/command``` and the message type ```CommandUR3e``` from ```ur3e_mrc.msg```, create a publisher to move the robot to desired joint angles. Keep in mind that the angles given to the robot should be in radians but we want to give the input in degrees so ensure that you have converted that. You can set the velocity and acceleration as `1.0`

The second step is to create a function (or multiple functions) in the same Python class to calculate the end effector pose using forward kinematics via DH-parameters, and print that out as the final transformation matrix.

Your code will have a structure like this (it can be different but just a baseline)

```python
import ....

class ForwardKinematicsUR3e(...)

  def __init__(self): 
    ...
    ...

  def move_robot(...):
    ...
    ...

  def calculate_fk_from_dh(...):
    ...
    ...

    
def main(...):

  ...
  ...

if __name__ == '__main__':
  main()
```

Hint: Use the structure from your ```pubsub``` codes which you have done previously. ~~You can get the message info for ```JointTrajectory``` and ```JointTrajectoryPoint``` here: http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html & http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html~~

Your command should look something like this:

```bash
ros2 run <package_name> ur3e_fk 0 0 0 0 0 0
```
where the numbers represent the six joint angles in degrees. Hint: Look into how you can send arguments to a Python script

Don't forget to add the node to your ```setup.py``` in your package. -->

## 4. Measure where the laser pointer reaches the table`



## 5. Compare the readings 

You need to compare the readings from the DH-parameters method with the actual robot position through ```/tf```. Put that in a table for the following 5 test cases:

Set 1: ```[0 0 0 0 0 0]``` (robot should be horizontal)

Set 2: ```[0 0 -90 90 0 0]```

Set 3: ```[0 -45 45 -90 30 90]```

Set 4: ```[90 -60 30 20 10 50]```

Set 5: ```[0 -90 0 0 0 0]``` (robot should be upright)

## Submission

No submissions for this week. However, make sure you collect the data for this week properly. There will be a joint submission with next week's studio covering the entire forward kinematics assignment.

<!-- 1. Show a screenshot of the base plate with the robot 

2. Show the DH Table for the robot

3. Show a figure with frames and axes marked

4. For each test case, show:

- The set of joint angle values (θ1, θ2, θ3, θ4, θ5, θ6)
- The final transformation matrix (from Python script). You can
add it as a readable image of the output window as well.
- The calculated pose from DH table in simulation vs the pose from ```/ur3/position```
- The scalar error

5. Discuss the sources of error

6. An appendix to show your scripts

- ```enme480_fk.xacro```
- FK publisher (including the Python script for DH transformation)
- ~~```tf``` subscriber~~ Screenshot of messages received from `/ur3/position`

Add everything in one single PDF file and upload it. -->






<!--
2. Interfacing the Robot with PC

Now that you've seen the teaching pendant we also want to demonstrate some of the same visualization tools you saw in the prior lab on the real robot. First, log into the ENME480 account on the computers (password ENME480) and locate the *commands2run.txt* file.  Open the file and follow the instructions within to allow the computer to control the robot. *Once this is done, the only control on the pendant which will do anything is the E-Stop button.* It is important whenever you are running code that one of your groupmates is holding the pendant and is ready to E-Stop if the robot moves unpredictably. 


3. Visualize ROS Processes on the Physical System


- Get the list of ROS topics
- Open up RQT
- Visualize Node Graphs
- You will be shown how to generate plots in RQT to analyze data
--!>





