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

## 🧪 **Verify Installation**

### **Test Basic Installation**
```bash
# Check ROS 2 version
ros2 --help

# List available packages
ros2 pkg list

# Run a simple demo
ros2 run demo_nodes_cpp talker
```

### **Test in New Terminal**
```bash
# Open new terminal and run
ros2 --help
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
