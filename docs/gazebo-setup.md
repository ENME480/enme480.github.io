# 🎮 Gazebo Setup Guide

<div align="center">

**Set up and configure Gazebo for robot simulation**

*Learn to use Gazebo for simulating robots, testing algorithms, and visualizing robot behavior*

</div>

---

## 🎯 **Overview**

Gazebo is a powerful 3D robot simulator that we'll use throughout ENME480 for testing robot control algorithms, visualizing robot movements, and simulating real-world scenarios. This guide covers installation, basic usage, and integration with ROS 2.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
- ✅ **Ubuntu 22.04 LTS** installed (ROS 2 Humble Tier-1 platform)
- ✅ **ROS 2 Humble** installed and configured
- ✅ **Graphics drivers** installed and working
- ✅ **At least 4GB RAM** (8GB recommended)

---

!!! tip "WSL/VM graphics"
    In WSLg and VMs (UTM), Gazebo's GUI can be slower. Install the proper **vGPU driver** on Windows (WSLg) or drop graphics quality / use headless sim as needed.

---

## 🚀 **Install Gazebo for ROS 2 Humble (Ubuntu 22.04)**

### **Option A — Gazebo Classic (gazebo11) + ROS interface (simple & stable)**
```bash
sudo apt update
sudo apt install -y gazebo    # installs Gazebo Classic (gazebo11) on Ubuntu 22.04
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### **Option B — Gazebo (GZ / Ignition) via ros_gz (newer stack)**
* For advanced users who want modern GZ features, see the official guide and `ros_gz` packages.
* Start here: [https://gazebosim.org/docs/latest/ros_installation/](https://gazebosim.org/docs/latest/ros_installation/) (migration notes: [https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/))

### **Verify Installation**
```bash
# Check Gazebo version
gazebo --version

# Launch Gazebo
gazebo
```

---

## 🔧 **Basic Gazebo Concepts**

### **World Files**
- **World files** (`.world`) define the simulation environment
- **Models** represent robots, objects, and environments
- **Physics engine** handles collisions and dynamics
- **Sensors** provide simulated sensor data

### **Key Components**
- **World**: 3D environment with physics
- **Models**: Robots, objects, buildings
- **Links**: Rigid bodies connected by joints
- **Joints**: Connections between links
- **Sensors**: Cameras, lasers, IMUs

---

## 🎮 **Launching Gazebo**

### **Launch from Terminal**
```bash
# Launch empty world
gazebo

# Launch specific world file
gazebo /usr/share/gazebo-11/worlds/empty.world

# Launch with specific physics settings
gazebo --physics-engine ode
```

### **Launch from ROS 2**
```bash
# Launch Gazebo with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py

# Launch with specific world
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.world
```

---

## 🏗️ **Creating a Simple World**

### **Basic World File**
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- A simple box -->
    <model name="box">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 0 0.5 0 0 0</pose>
    </model>
  </world>
</sdf>
```

### **Save and Load World**
```bash
# Save world file
# Copy the XML above to ~/gazebo_worlds/simple.world

# Launch custom world
gazebo ~/gazebo_worlds/simple.world
```

---

## 🤖 **Adding Robot Models**

### **Install UR3e Model**
```bash
# Clone UR3e model repository
cd ~/ros2_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

# Build workspace
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash
```

### **Launch UR3e in Gazebo**
```bash
# Launch UR3e in Gazebo
ros2 launch ur_gazebo ur3e_bringup.launch.py

# Or launch with custom world
ros2 launch ur_gazebo ur3e_bringup.launch.py world:=~/gazebo_worlds/simple.world
```

---

## 🎯 **Basic Gazebo Operations**

### **Camera Controls**
- **Mouse**: Rotate view
- **Scroll wheel**: Zoom in/out
- **Right-click + drag**: Pan view
- **Middle-click + drag**: Rotate around point

### **Model Manipulation**
- **Select model**: Click on it
- **Move model**: Drag with left mouse button
- **Rotate model**: Drag with right mouse button
- **Scale model**: Use the scale handles

### **Time Controls**
- **Play/Pause**: Spacebar
- **Step**: Step button in toolbar
- **Reset**: Reset button in toolbar
- **Real-time factor**: Adjust in toolbar

---

## 🔍 **Using Gazebo GUI**

### **Main Toolbar**
- **Play/Pause**: Start/stop simulation
- **Step**: Advance simulation one step
- **Reset**: Reset simulation to initial state
- **Real-time factor**: Speed up/slow down simulation

### **Left Panel**
- **World**: View world hierarchy
- **Models**: List all models in world
- **Joints**: View joint properties
- **Sensors**: Configure sensors

### **Right Panel**
- **Properties**: Edit selected object properties
- **Material**: Change object appearance
- **Physics**: Adjust physics properties

---

## 📡 **ROS 2 Integration**

### **Gazebo ROS 2 Plugins**
```bash
# Install Gazebo ROS 2 plugins
sudo apt install ros-humble-gazebo-ros-pkgs

# Check available plugins
gazebo --help
```

### **Launch File Example**
```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Declare launch arguments
    world_file = LaunchConfiguration('world')
    
    # Launch Gazebo
    gazebo = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
        arguments=[world_file]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world'),
            description='Path to world file'
        ),
        gazebo
    ])
```

---

## 🧪 **Basic Simulation Examples**

### **Example 1: Simple Robot Movement**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class UR3eController(Node):
    def __init__(self):
        super().__init__('ur3e_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        
        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Timer for periodic commands
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('UR3e controller started')
    
    def joint_callback(self, msg):
        """Callback for joint state updates."""
        if len(msg.position) >= 6:
            self.get_logger().info(f'Joint angles: {[f"{x:.2f}" for x in msg.position[:6]]}')
    
    def timer_callback(self):
        """Send periodic joint commands."""
        # Simple sinusoidal movement
        import math
        import time
        
        t = time.time()
        joint_angles = [
            0.0,  # Base
            math.sin(t) * 0.5,  # Shoulder
            math.cos(t) * 0.5,  # Elbow
            0.0,  # Wrist 1
            0.0,  # Wrist 2
            0.0   # Wrist 3
        ]
        
        joint_msg = Float64MultiArray()
        joint_msg.data = joint_angles
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR3eController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Example 2: Camera Visualization**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Subscribe to camera topic
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('Camera viewer started')
    
    def camera_callback(self, msg):
        """Display camera image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔧 **Troubleshooting Common Issues**

### **Performance Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Slow simulation** | Reduce physics update rate, use simpler models |
| **High CPU usage** | Close other applications, reduce model complexity |
| **Memory issues** | Use fewer models, restart Gazebo |

### **Graphics Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Black screen** | Check graphics drivers, try software rendering |
| **Low FPS** | Reduce graphics quality, use simpler models |
| **Model not visible** | Check model URDF, verify file paths |

### **ROS 2 Integration Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Topics not appearing** | Check launch file, verify package installation |
| **Models not loading** | Check URDF files, verify package paths |
| **Physics not working** | Check physics engine, verify model collision |

---

## 📚 **Advanced Features**

### **Custom Models**
```bash
# Create model directory
mkdir -p ~/.gazebo/models/my_robot

# Create model.config file
cat > ~/.gazebo/models/my_robot/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>My custom robot model</description>
</model>
EOF

# Create model.sdf file (similar to world file)
# ... create SDF file ...
```

### **Physics Parameters**
```xml
<!-- In world file -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
```

---

## ✅ **Verification Checklist**

- [ ] Gazebo launches successfully
- [ ] ROS 2 integration working
- [ ] UR3e model loads correctly
- [ ] Basic simulation runs
- [ ] Camera visualization working
- [ ] Joint control functional
- [ ] Physics simulation realistic

---

## 🆘 **Getting Help**

### **Gazebo Resources**
- **Official Docs**: [gazebosim.org](http://gazebosim.org/tutorials)
- **ROS 2 Integration**: [docs.ros.org](https://docs.ros.org/en/humble/Guides/Gazebo.html)
- **Community Forum**: [answers.gazebosim.org](https://answers.gazebosim.org/)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

## 🚀 **Next Steps**

After setting up Gazebo:

1. **Practice basic operations** in empty world
2. **Load and control UR3e** robot
3. **Start Week 4 lab**: See [Week 4 Lab](labs/week-04.md)
4. **Experiment with different worlds** and models

---

<div align="center">

**Ready to simulate robots? Let's start the Week 4 lab! 🎮**

[🐍 Python Basics](python-basics.md){ .md-button }
[🤖 ROS Setup](ros-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
