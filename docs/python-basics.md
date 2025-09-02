# 🐍 Python Basics for Robotics

<div align="center">

**Essential Python programming for robotics development**

*Learn the Python fundamentals you'll need for ROS 2, robot control, and simulation*

</div>

---

## 🎯 **Overview**

This guide covers essential Python concepts for robotics programming. You'll learn the basics needed to write ROS 2 nodes, control robots, process sensor data, and work with mathematical operations.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
- ✅ **Ubuntu with Python 3.8+** installed
- ✅ **Basic terminal knowledge**
- ✅ **Text editor** (VS Code, gedit, or nano)

---

## 🚀 **Getting Started**

### **Check Python Installation**
```bash
# Check Python version
python3 --version

# Check pip version
pip3 --version

# Start Python interpreter
python3
```

### **Install Essential Packages**
```bash
# Install common robotics packages
pip3 install numpy matplotlib scipy

# Install ROS 2 Python client
sudo apt install python3-rclpy
```

---

## 📚 **Core Python Concepts**

### **1. Variables and Data Types**
```python
# Numbers
x = 10          # integer
y = 3.14        # float
z = 2 + 3j      # complex

# Strings
name = "Robot"
message = 'Hello, World!'

# Booleans
is_robot = True
is_human = False

# Lists (mutable)
joint_angles = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]
joint_names = ["shoulder", "elbow", "wrist"]

# Tuples (immutable)
position = (1.0, 2.0, 3.0)

# Dictionaries
robot_config = {
    "name": "UR3e",
    "dof": 6,
    "max_payload": 3.0
}
```

### **2. Control Flow**
```python
# If statements
if joint_angle > 1.57:
    print("Joint limit exceeded!")
elif joint_angle < -1.57:
    print("Joint limit exceeded!")
else:
    print("Joint angle is safe")

# For loops
for i in range(6):
    print(f"Joint {i}: {joint_angles[i]}")

# While loops
count = 0
while count < 5:
    print(f"Count: {count}")
    count += 1
```

### **3. Functions**
```python
def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    import math
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    dz = point2[2] - point1[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

# Function with default parameters
def move_robot(x=0.0, y=0.0, z=0.0, speed=1.0):
    """Move robot to specified position."""
    print(f"Moving to ({x}, {y}, {z}) at speed {speed}")
    return True

# Call functions
distance = calculate_distance((0, 0, 0), (1, 1, 1))
move_robot(1.0, 2.0, 3.0)
```

---

## 🔢 **Mathematics and NumPy**

### **NumPy Basics**
```python
import numpy as np

# Create arrays
joint_angles = np.array([0.0, 1.57, 0.0, 0.0, 0.0, 0.0])
position = np.array([1.0, 2.0, 3.0])

# Array operations
angles_deg = np.degrees(joint_angles)
angles_rad = np.radians(angles_deg)

# Matrix operations
rotation_matrix = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

# Matrix multiplication
new_position = rotation_matrix @ position
```

### **Common Mathematical Operations**
```python
import numpy as np
import math

# Trigonometric functions
angle = np.pi / 4
sin_val = np.sin(angle)
cos_val = np.cos(angle)
tan_val = np.tan(angle)

# Square root and power
distance = np.sqrt(16)
squared = np.power(4, 2)

# Random numbers
random_angle = np.random.uniform(-np.pi, np.pi)
random_position = np.random.randn(3)  # 3D normal distribution
```

---

## 📊 **Data Visualization with Matplotlib**

### **Basic Plotting**
```python
import matplotlib.pyplot as plt
import numpy as np

# Create data
time = np.linspace(0, 10, 100)
joint_angle = np.sin(time)

# Create plot
plt.figure(figsize=(10, 6))
plt.plot(time, joint_angle, 'b-', linewidth=2, label='Joint Angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Joint Angle vs Time')
plt.grid(True)
plt.legend()
plt.show()
```

### **Multiple Plots**
```python
# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# First subplot
ax1.plot(time, np.sin(time), 'r-', label='Sine')
ax1.set_ylabel('Amplitude')
ax1.legend()
ax1.grid(True)

# Second subplot
ax2.plot(time, np.cos(time), 'g-', label='Cosine')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Amplitude')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()
```

---

## 🤖 **ROS 2 Integration**

### **Basic ROS 2 Node**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publisher
        self.joint_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_commands', 
            10
        )
        
        # Create subscriber
        self.pose_sub = self.create_subscription(
            Pose,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Robot controller node started')
    
    def pose_callback(self, msg):
        """Callback for robot pose updates."""
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        self.get_logger().info(f'Robot at: ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def timer_callback(self):
        """Timer callback for periodic tasks."""
        # Send joint commands
        joint_msg = Float64MultiArray()
        joint_msg.data = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔧 **File I/O and Data Handling**

### **Reading and Writing Files**
```python
# Write data to file
joint_data = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]

with open('joint_angles.txt', 'w') as f:
    for angle in joint_data:
        f.write(f"{angle}\n")

# Read data from file
angles = []
with open('joint_angles.txt', 'r') as f:
    for line in f:
        angles.append(float(line.strip()))

print(f"Read angles: {angles}")
```

### **CSV Files**
```python
import csv

# Write CSV
with open('robot_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Time', 'Joint1', 'Joint2', 'Joint3'])
    writer.writerow([0.0, 0.0, 1.57, 0.0])
    writer.writerow([0.1, 0.1, 1.67, 0.1])

# Read CSV
with open('robot_data.csv', 'r') as f:
    reader = csv.reader(f)
    header = next(reader)  # Skip header
    for row in reader:
        time, j1, j2, j3 = float(row[0]), float(row[1]), float(row[2]), float(row[3])
        print(f"Time: {time}, Joints: [{j1}, {j2}, {j3}]")
```

---

## 🧪 **Testing and Debugging**

### **Basic Testing**
```python
def test_calculate_distance():
    """Test the calculate_distance function."""
    # Test case 1: Distance from origin
    result = calculate_distance((0, 0, 0), (1, 1, 1))
    expected = np.sqrt(3)
    assert abs(result - expected) < 1e-6, f"Expected {expected}, got {result}"
    
    # Test case 2: Same point
    result = calculate_distance((1, 2, 3), (1, 2, 3))
    expected = 0.0
    assert abs(result - expected) < 1e-6, f"Expected {expected}, got {result}"
    
    print("All tests passed!")

# Run tests
test_calculate_distance()
```

### **Debugging Tips**
```python
# Use print statements
print(f"Debug: joint_angles = {joint_angles}")

# Use pdb debugger
import pdb
pdb.set_trace()  # Code stops here for debugging

# Use logging
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)
logger.debug(f"Joint angles: {joint_angles}")
```

---

## 📱 **Common Robotics Patterns**

### **State Machine**
```python
class RobotState:
    IDLE = "IDLE"
    MOVING = "MOVING"
    ERROR = "ERROR"

class RobotController:
    def __init__(self):
        self.state = RobotState.IDLE
        self.target_position = None
    
    def update(self):
        if self.state == RobotState.IDLE:
            if self.target_position:
                self.state = RobotState.MOVING
                print("Starting movement...")
        
        elif self.state == RobotState.MOVING:
            if self.reached_target():
                self.state = RobotState.IDLE
                print("Movement completed")
        
        elif self.state == RobotState.ERROR:
            print("Robot in error state")
    
    def reached_target(self):
        # Simulate reaching target
        return True
```

### **PID Controller**
```python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Update previous error
        self.prev_error = error
        
        # Compute output
        output = p_term + i_term + d_term
        return output

# Usage example
pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
control_output = pid.compute(setpoint=1.0, measurement=0.5, dt=0.01)
```

---

## ✅ **Practice Exercises**

### **Exercise 1: Joint Limit Checker**
```python
def check_joint_limits(joint_angles, limits):
    """
    Check if joint angles are within limits.
    
    Args:
        joint_angles: List of joint angles
        limits: List of (min, max) tuples for each joint
    
    Returns:
        List of booleans indicating if each joint is within limits
    """
    # Your code here
    pass

# Test data
joints = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]
limits = [(-3.14, 3.14)] * 6  # All joints have same limits

# Test your function
results = check_joint_limits(joints, limits)
print(f"Joint limits check: {results}")
```

### **Exercise 2: Trajectory Generator**
```python
def generate_trajectory(start_pos, end_pos, num_points):
    """
    Generate a linear trajectory between two positions.
    
    Args:
        start_pos: Starting position (x, y, z)
        end_pos: Ending position (x, y, z)
        num_points: Number of points in trajectory
    
    Returns:
        List of positions along the trajectory
    """
    # Your code here
    pass

# Test your function
start = (0, 0, 0)
end = (1, 1, 1)
trajectory = generate_trajectory(start, end, 10)
print(f"Trajectory: {trajectory}")
```

---

## 🆘 **Getting Help**

### **Python Resources**
- **Official Docs**: [python.org](https://docs.python.org/3/)
- **NumPy Docs**: [numpy.org](https://numpy.org/doc/)
- **Matplotlib Docs**: [matplotlib.org](https://matplotlib.org/)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

## 🚀 **Next Steps**

After mastering Python basics:

1. **Set up ROS 2**: See [ROS Setup Guide](ros-setup.md)
2. **Learn Gazebo**: See [Gazebo Setup](gazebo-setup.md)
3. **Start Week 3 lab**: See [Week 3 Lab](labs/week-03.md)
4. **Practice coding**: Work on exercises and small projects

---

<div align="center">

**Ready to control robots? Let's set up ROS 2 next! 🤖**

[🤖 ROS Setup](ros-setup.md){ .md-button .md-button--primary }
[🎮 Gazebo Setup](gazebo-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
