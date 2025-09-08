# 🤖 ROS 2 Setup Guide

<div align="center">

**Install and configure ROS 2 Humble for robotics development**

*Get ROS 2 running on Ubuntu for ENME480 labs and projects*

</div>

---

## 🎯 **Overview**

This guide will help you install ROS 2 Humble (Hawksbill) on Ubuntu 22.04. ROS 2 is the Robot Operating System that we'll use throughout the course for robot programming and simulation.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
- ✅ **Ubuntu 22.04 LTS** installed (ROS 2 Humble Tier-1 platform)
- ✅ **Internet connection** for downloading packages
- ✅ **Basic Ubuntu knowledge** (terminal commands)
- ✅ **At least 10GB free space**

---
## Windows 10/11

1. First, we will make sure our dependencies are in place. Within WSL2 run:
   
        sudo apt update && sudo apt upgrade
   In order to update all system packages (you may need to enter your password)
   
         sudo install -m 0755 -d /etc/apt/keyrings && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg && sudo chmod a+r /etc/apt/keyrings/docker.gpg

        sudo apt update && sudo apt upgrade -y

        sudo apt install -y apt-transport-https ca-certificates curl software-properties-common

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

        echo "deb [signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

        sudo apt update
These commands set up package registries within WSL, which is how Ubuntu knows where to look for packages (apps) we want to install. If you'd like a more detailed breakdown of what each command here does, feel free to ask a TA.

        sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin git python-is-python3 docker
This command will install docker (which we use to standarize everyones ROS installation), git (which we use to sync code across computers) and remaps the name "python3" to "python" to make Ubuntu happier when running our code. 

2. Now, we will download the code from GitHub. To do this, run:
   
         cd ~/ && git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc.git

To move to the right folder and download the GitHub repo containing the docker image we need.

3. With that done, we need to make sure the user groups are set up to allow us to compile and run docker images. Run:

        sudo groupadd docker 
        
        sudo usermod -aG docker $USER 
        
        newgrp docker

        sudo systemctl restart docker
So that you are able to build and run docker images. These commands make a gruop who can manage docker images, then add you to it, then resets part of Ubuntu so it recognizes the new group. Once this is done, all the parts are in place to build our docker image.

4. Now, we will build our image.

        cd ~/ENME480_mrc/docker && userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
The first part of this command (before the &&) puts you in the folder containing the docker image we want to build, while the second part actually builds our image. This step can take a while, since you have to download a lot of data. If you get a permission error at this step try restarting wsl.

Next, try running

        nvidia-smi
You should get an output which looks something like

*If you do, follow the next step, if not skip to step 7.*

5. Getting the an output from nvidia-smi means you have a Nvidia GPU installed in your computer with drivers properly configured. In this step, we will enable the GPU within docker to speed up our simulations. First, run the following commands:

        sudo touch /etc/docker/daemon.json
	
		sudo chmod 777 /etc/docker/daemon.json
	
		curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
	          && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
	            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
	            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
		
		sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
		
		sudo apt-get update
	
        export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
        sudo apt-get install -y \
              nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
This will install the Nvidia container toolkit which allows Docker to use your GPU.
6. With the container toolkit installed, we can now configure docker and compose our image. 

	echo $'{"runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}}}' > /etc/docker/daemon.json && sudo systemctl restart docker
 This command will add a line to the settings file to enable running with the Nvidia GPU then resets Docker to reload the configuration.

	docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
Finally, this command will compose and run our image. This is the command you will want to run in order to get into the Docker and use ROS. Once it finishes you should see that the username in the terminal will have changed to "enme480_docker" to let you know that you are in the docker container. You can skip step 9 if you've done this, step 7 is for people not running with Nvidia GPUs. From here, you can go to the VSCode setup or continue to set up what ever IDE you'd like to use.
7. If you are not running with an Nvidia GPU you can skip setting up the Nvidia toolkit and instead just run:

	docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
This is the command you will need to run to enter the Docker and use ROS. The next step is to configure what ever IDE you'd like to use. We recommend VSCode for it's Docker integration, but you are free to use any IDE you'd like.

## Ubuntu (22.04 LTS)
Assuming you are using a fresh install, start from Step 4 in the Windows section. If you are using an installation you've customized you should check what of the prerequisites are already installed to avoid causing dependency issues with other programs. **Warning: If you are using a snap installation of Docker the step to configure docker to use Nvidia Container Toolkit is different, we strongly recommend using the apt installation.** 

Setting up VSCode is also very similar. The only difference is that if you don't want to download the deb package from the microsoft site you can use either apt or snap to get the package via:

	sudo <apt/snap> install code (or code-insiders)



---
## 🧪 **Verify Installation**

### **Test Basic Installation**
From within the docker image, run the following command:
```
ros2 run demo_nodes_cpp talker
```
This shouuld begin outputting a list of number to the terminal. Open a new terminal, enter the docer image and run:
```
ros2 run demo_nodes_cpp listener
```
This second script should output the messages being sent by the talker.
### **Test in New Terminal**
```
# Open new terminal and run
ros2 --help
# test gazebo, our simulation suite
ign gazebo
```

---

## 🔧 **Common Issues & Solutions**

### **Installation Problems**
| **Error** | **Solution** |
|-----------|--------------|
| **GPG error** | Re-run GPG key commands |
| **Package not found** | Check Ubuntu version compatibility |
| **Permission denied** | Use `sudo` for system commands |

### **Environment Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Command not found** | Source setup.bash or restart terminal |
| **Package not found** | Check if package is installed |
| **Version mismatch** | Ensure Ubuntu and ROS versions match |

---

## 📚 **Essential ROS 2 Concepts**

### **Core Concepts**
- **Nodes**: Individual processes that perform computation
- **Topics**: Asynchronous communication mechanism
- **Services**: Synchronous request-response communication
- **Actions**: Long-running tasks with feedback
- **Messages**: Data structures for communication

### **Basic Commands**
```bash
# List running nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# List services
ros2 service list

# Call a service
ros2 service call /service_name service_type "data"
```

---

## 🛠️ **Development Setup**

### **Create ROS 2 Workspace**
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Add to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### **Install Useful Tools**
```bash
# Install rqt (GUI tools)
sudo apt install ros-humble-rqt

# Install rviz2 (3D visualization)
sudo apt install ros-humble-rviz2

# Install turtlesim (tutorial package)
sudo apt install ros-humble-turtlesim
```

---

## 🎮 **First ROS 2 Program**

### **Create a Simple Publisher**
```bash
# Navigate to workspace
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_cmake my_first_pkg

# Navigate to package
cd my_first_pkg/src

# Create Python file
touch my_first_node.py
```

### **Add Code to my_first_node.py**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from my first ROS 2 node!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Build and Run**
```bash
# Build workspace
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash

# Run node
ros2 run my_first_pkg my_first_node
```

---

## 🔍 **Debugging & Troubleshooting**

### **Useful Debugging Commands**
```bash
# Check node status
ros2 node info /node_name

# Monitor topic frequency
ros2 topic hz /topic_name

# Check message type
ros2 topic type /topic_name

# List node parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name
```

### **Common Debugging Steps**
1. **Check if node is running**: `ros2 node list`
2. **Verify topic exists**: `ros2 topic list`
3. **Check message data**: `ros2 topic echo /topic_name`
4. **Monitor system resources**: `htop`, `ros2 topic hz`

---

## 📱 **ROS 2 Tools & GUIs**

### **Command Line Tools**
- **ros2**: Main command line interface
- **ros2 topic**: Topic management
- **ros2 service**: Service management
- **ros2 node**: Node management
- **ros2 param**: Parameter management

### **Graphical Tools**
- **rqt**: Plugin-based GUI framework
- **rviz2**: 3D visualization tool
- **plotjuggler**: Data plotting and analysis
- **rqt_graph**: Node and topic visualization

---

## ✅ **Verification Checklist**

- [ ] ROS 2 Humble installed successfully
- [ ] Environment sourced correctly
- [ ] Basic commands working (`ros2 --help`)
- [ ] Demo nodes running (`ros2 run demo_nodes_cpp talker`)
- [ ] Workspace created and building
- [ ] First custom node working
- [ ] Tools installed (rqt, rviz2)

---

## 🆘 **Getting Help**

### **ROS 2 Resources**
- **Official Docs**: [docs.ros.org](https://docs.ros.org/en/humble/)
- **ROS Answers**: [answers.ros.org](https://answers.ros.org/)
- **ROS Wiki**: [wiki.ros.org](https://wiki.ros.org/)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

## 🚀 **Next Steps**

After completing ROS 2 setup:

1. **Learn Python basics**: See [Python Basics](python-basics.md)
2. **Set up Gazebo**: See [Gazebo Setup](gazebo-setup.md)
3. **Start Week 3 lab**: See [Week 3 Lab](labs/week-03.md)
4. **Practice ROS 2**: Run tutorials and examples

---

<div align="center">

**Ready to start programming robots? Let's learn Python next! 🐍**

[🐍 Python Basics](python-basics.md){ .md-button .md-button--primary }
[🎮 Gazebo Setup](gazebo-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
