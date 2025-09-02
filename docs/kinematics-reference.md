# 📐 Kinematics Reference

<div align="center">

**Essential kinematics formulas and reference materials**

*DH parameters, transformation matrices, forward/inverse kinematics for robotics*

</div>

---

## 🎯 **Overview**

This reference guide contains essential kinematics formulas, DH parameters, and mathematical tools you'll need for ENME480. Keep this handy during labs and homework assignments.

---

## 🔢 **Mathematical Fundamentals**

### **Rotation Matrices**
```python
import numpy as np

# Rotation around X-axis (roll)
def Rx(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

# Rotation around Y-axis (pitch)
def Ry(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

# Rotation around Z-axis (yaw)
def Rz(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# Combined rotation (ZYX Euler angles)
def Rzyx(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)
```

### **Homogeneous Transformations**
```python
def homogeneous_transform(R, p):
    """Create 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def translation_matrix(x, y, z):
    """Create translation matrix."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def rotation_matrix(R):
    """Create rotation matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    return T
```

---

## 🤖 **DH Parameters**

### **DH Parameter Table**
| **Joint** | **θ** | **d** | **a** | **α** |
|-----------|-------|-------|-------|--------|
| **Base** | θ₁ | d₁ | a₁ | α₁ |
| **Shoulder** | θ₂ | d₂ | a₂ | α₂ |
| **Elbow** | θ₃ | d₃ | a₃ | α₃ |
| **Wrist 1** | θ₄ | d₄ | a₄ | α₄ |
| **Wrist 2** | θ₅ | d₅ | a₅ | α₅ |
| **Wrist 3** | θ₆ | d₆ | a₆ | α₆ |

### **DH Transformation Matrix**
```python
def dh_transform(theta, d, a, alpha):
    """Compute DH transformation matrix."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    
    return T
```

---

## 🦾 **UR3e DH Parameters**

### **Standard DH Parameters**
```python
# UR3e DH parameters (in meters and radians)
ur3e_dh = [
    # [theta, d, a, alpha]
    [0, 0.1519, 0, -np.pi/2],      # Base
    [0, 0, -0.24365, 0],           # Shoulder
    [0, 0, -0.21325, 0],           # Elbow
    [0, 0.11235, 0, -np.pi/2],     # Wrist 1
    [0, 0.08535, 0, np.pi/2],      # Wrist 2
    [0, 0.0819, 0, 0]              # Wrist 3
]

# Joint limits (in radians)
ur3e_limits = [
    [-2*np.pi, 2*np.pi],    # Base
    [-2*np.pi, 2*np.pi],    # Shoulder
    [-np.pi, 0],             # Elbow
    [-2*np.pi, 2*np.pi],    # Wrist 1
    [-2*np.pi, 2*np.pi],    # Wrist 2
    [-2*np.pi, 2*np.pi]     # Wrist 3
]
```

### **Modified DH Parameters**
```python
# Modified DH parameters (alternative convention)
ur3e_mdh = [
    # [theta, d, a, alpha]
    [0, 0.1519, 0, 0],             # Base
    [0, 0, 0.24365, -np.pi/2],     # Shoulder
    [0, 0, 0.21325, 0],            # Elbow
    [0, 0.11235, 0, -np.pi/2],     # Wrist 1
    [0, 0.08535, 0, np.pi/2],      # Wrist 2
    [0, 0.0819, 0, 0]              # Wrist 3
]
```

---

## ➡️ **Forward Kinematics**

### **Basic Forward Kinematics**
```python
def forward_kinematics(joint_angles, dh_params):
    """Compute forward kinematics using DH parameters."""
    T = np.eye(4)
    
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        # Add joint angle to theta
        current_theta = theta + joint_angles[i]
        
        # Compute transformation for this joint
        Ti = dh_transform(current_theta, d, a, alpha)
        
        # Multiply transformations
        T = T @ Ti
    
    return T

# Example usage
joint_angles = [0, -np.pi/2, 0, 0, 0, 0]  # Home position
T_end_effector = forward_kinematics(joint_angles, ur3e_dh)

# Extract position and orientation
position = T_end_effector[:3, 3]
orientation = T_end_effector[:3, :3]
```

### **Position and Orientation Extraction**
```python
def extract_pose(T):
    """Extract position and orientation from transformation matrix."""
    position = T[:3, 3]
    orientation = T[:3, :3]
    
    # Convert to Euler angles (ZYX convention)
    yaw = np.arctan2(orientation[1, 0], orientation[0, 0])
    pitch = np.arctan2(-orientation[2, 0], 
                       np.sqrt(orientation[2, 1]**2 + orientation[2, 2]**2))
    roll = np.arctan2(orientation[2, 1], orientation[2, 2])
    
    return position, np.array([roll, pitch, yaw])

def extract_quaternion(T):
    """Extract quaternion from transformation matrix."""
    from scipy.spatial.transform import Rotation as R
    
    orientation = T[:3, :3]
    r = R.from_matrix(orientation)
    return r.as_quat()  # [x, y, z, w]
```

---

## ⬅️ **Inverse Kinematics**

### **Analytical IK for UR3e**
```python
def ur3e_inverse_kinematics(T_desired, elbow_config=1):
    """
    Analytical inverse kinematics for UR3e.
    
    Args:
        T_desired: Desired end-effector pose (4x4 matrix)
        elbow_config: 1 for elbow up, -1 for elbow down
    
    Returns:
        List of joint angles (6x1)
    """
    # Extract position and orientation
    p_desired = T_desired[:3, 3]
    R_desired = T_desired[:3, :3]
    
    # DH parameters
    d1, d4, d5, d6 = 0.1519, 0.11235, 0.08535, 0.0819
    a2, a3 = 0.24365, 0.21325
    
    # Wrist center position
    p_wrist = p_desired - d6 * R_desired[:, 2]
    
    # Joint 1 (base rotation)
    theta1 = np.arctan2(p_wrist[1], p_wrist[0])
    
    # Joint 2 and 3 (shoulder and elbow)
    r = np.sqrt(p_wrist[0]**2 + p_wrist[1]**2)
    s = p_wrist[2] - d1
    
    # Cosine law
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    
    if abs(D) > 1:
        raise ValueError("Target position not reachable")
    
    theta3 = elbow_config * np.arccos(D)
    theta2 = np.arctan2(s, r) - np.arctan2(a3 * np.sin(theta3), 
                                           a2 + a3 * np.cos(theta3))
    
    # Joints 4, 5, 6 (wrist)
    R03 = forward_kinematics([theta1, theta2, theta3, 0, 0, 0], ur3e_dh)[:3, :3]
    R36 = R03.T @ R_desired
    
    # Extract Euler angles from R36
    theta4 = np.arctan2(R36[1, 2], R36[0, 2])
    theta5 = np.arctan2(np.sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2])
    theta6 = np.arctan2(-R36[2, 1], R36[2, 0])
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]

# Example usage
T_desired = np.eye(4)
T_desired[:3, 3] = [0.3, 0.0, 0.5]  # Desired position

try:
    joint_angles = ur3e_inverse_kinematics(T_desired, elbow_config=1)
    print(f"Joint angles: {[f'{x:.3f}' for x in joint_angles]}")
except ValueError as e:
    print(f"Error: {e}")
```

### **Numerical IK (Jacobian Method)**
```python
def numerical_inverse_kinematics(T_desired, initial_guess, max_iter=100, tol=1e-6):
    """Numerical inverse kinematics using Jacobian method."""
    from scipy.optimize import minimize
    
    def objective_function(joint_angles):
        T_current = forward_kinematics(joint_angles, ur3e_dh)
        
        # Position error
        pos_error = np.linalg.norm(T_current[:3, 3] - T_desired[:3, 3])
        
        # Orientation error (simplified)
        ori_error = np.linalg.norm(T_current[:3, :3] - T_desired[:3, :3])
        
        return pos_error + 0.1 * ori_error
    
    # Minimize objective function
    result = minimize(objective_function, initial_guess, 
                     method='L-BFGS-B', 
                     bounds=ur3e_limits,
                     options={'maxiter': max_iter})
    
    if result.success:
        return result.x
    else:
        raise ValueError("IK optimization failed")

# Example usage
initial_guess = [0, -np.pi/2, 0, 0, 0, 0]
try:
    joint_angles = numerical_inverse_kinematics(T_desired, initial_guess)
    print(f"Numerical IK result: {[f'{x:.3f}' for x in joint_angles]}")
except ValueError as e:
    print(f"Error: {e}")
```

---

## 🔄 **Jacobian Matrix**

### **Analytical Jacobian**
```python
def compute_jacobian(joint_angles, dh_params):
    """Compute analytical Jacobian matrix."""
    n_joints = len(dh_params)
    J = np.zeros((6, n_joints))
    
    # Current end-effector pose
    T_current = forward_kinematics(joint_angles, dh_params)
    p_current = T_current[:3, 3]
    
    # Compute Jacobian columns
    for i in range(n_joints):
        # Perturb joint i
        joint_angles_perturbed = joint_angles.copy()
        joint_angles_perturbed[i] += 1e-6
        
        # Compute perturbed pose
        T_perturbed = forward_kinematics(joint_angles_perturbed, dh_params)
        p_perturbed = T_perturbed[:3, 3]
        
        # Linear velocity component
        J[:3, i] = (p_perturbed - p_current) / 1e-6
        
        # Angular velocity component (simplified)
        # For full implementation, compute orientation differences
    
    return J

# Example usage
joint_angles = [0, -np.pi/2, 0, 0, 0, 0]
J = compute_jacobian(joint_angles, ur3e_dh)
print(f"Jacobian shape: {J.shape}")
```

### **Jacobian-Based Control**
```python
def jacobian_control(joint_angles, desired_velocity, dh_params, dt=0.01):
    """Simple Jacobian-based velocity control."""
    J = compute_jacobian(joint_angles, dh_params)
    
    # Pseudo-inverse of Jacobian
    J_inv = np.linalg.pinv(J)
    
    # Joint velocities
    joint_velocities = J_inv @ desired_velocity
    
    # Update joint angles
    new_joint_angles = joint_angles + joint_velocities * dt
    
    return new_joint_angles

# Example usage
desired_velocity = np.array([0.1, 0, 0, 0, 0, 0])  # Move in X direction
new_angles = jacobian_control(joint_angles, desired_velocity, ur3e_dh)
```

---

## 📊 **Trajectory Generation**

### **Linear Trajectory**
```python
def linear_trajectory(start_pos, end_pos, num_points):
    """Generate linear trajectory between two points."""
    t = np.linspace(0, 1, num_points)
    
    trajectory = []
    for ti in t:
        pos = start_pos + ti * (end_pos - start_pos)
        trajectory.append(pos)
    
    return np.array(trajectory)

# Example usage
start_pos = np.array([0.3, 0.0, 0.5])
end_pos = np.array([0.5, 0.2, 0.3])
trajectory = linear_trajectory(start_pos, end_pos, 50)
```

### **Joint Space Trajectory**
```python
def joint_trajectory(start_angles, end_angles, num_points):
    """Generate trajectory in joint space."""
    t = np.linspace(0, 1, num_points)
    
    trajectory = []
    for ti in t:
        angles = start_angles + ti * (np.array(end_angles) - np.array(start_angles))
        trajectory.append(angles)
    
    return np.array(trajectory)

# Example usage
start_angles = [0, -np.pi/2, 0, 0, 0, 0]
end_angles = [np.pi/4, -np.pi/3, -np.pi/6, 0, 0, 0]
joint_traj = joint_trajectory(start_angles, end_angles, 100)
```

---

## 🔧 **Utility Functions**

### **Distance and Angle Calculations**
```python
def euclidean_distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(p1) - np.array(p2))

def angle_between_vectors(v1, v2):
    """Calculate angle between two vectors."""
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_angle = np.clip(cos_angle, -1, 1)  # Avoid numerical errors
    return np.arccos(cos_angle)

def normalize_angle(angle):
    """Normalize angle to [-π, π]."""
    return np.arctan2(np.sin(angle), np.cos(angle))
```

### **Matrix Operations**
```python
def skew_symmetric(v):
    """Create skew-symmetric matrix from vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def rotation_matrix_to_axis_angle(R):
    """Convert rotation matrix to axis-angle representation."""
    theta = np.arccos((np.trace(R) - 1) / 2)
    
    if abs(theta) < 1e-6:
        return np.array([0, 0, 1]), 0
    
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    axis = axis / (2 * np.sin(theta))
    
    return axis, theta
```

---

## 📚 **Common Formulas**

### **Trigonometric Identities**
```python
# Sum and difference formulas
def sin_sum(a, b):
    return np.sin(a) * np.cos(b) + np.cos(a) * np.sin(b)

def cos_sum(a, b):
    return np.cos(a) * np.cos(b) - np.sin(a) * np.sin(b)

# Double angle formulas
def sin_double(a):
    return 2 * np.sin(a) * np.cos(a)

def cos_double(a):
    return np.cos(a)**2 - np.sin(a)**2
```

### **Vector Operations**
```python
def cross_product(v1, v2):
    """Cross product of two 3D vectors."""
    return np.array([
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    ])

def dot_product(v1, v2):
    """Dot product of two vectors."""
    return np.sum(v1 * v2)
```

---

## ✅ **Verification Examples**

### **Test Forward Kinematics**
```python
def test_forward_kinematics():
    """Test forward kinematics with known values."""
    # Test home position
    home_angles = [0, -np.pi/2, 0, 0, 0, 0]
    T_home = forward_kinematics(home_angles, ur3e_dh)
    
    print("Home position:")
    print(f"Position: {T_home[:3, 3]}")
    print(f"Orientation:\n{T_home[:3, :3]}")
    
    # Test with different angles
    test_angles = [np.pi/4, -np.pi/3, -np.pi/6, 0, 0, 0]
    T_test = forward_kinematics(test_angles, ur3e_dh)
    
    print("\nTest position:")
    print(f"Position: {T_test[:3, 3]}")
    print(f"Orientation:\n{T_test[:3, :3]}")

# Run test
test_forward_kinematics()
```

---

## 🆘 **Getting Help**

### **Mathematical Resources**
- **Linear Algebra**: [Khan Academy](https://www.khanacademy.org/math/linear-algebra)
- **Robotics Math**: [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- **Python Math**: [SciPy Documentation](https://docs.scipy.org/doc/scipy/reference/)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

## 🚀 **Next Steps**

After mastering kinematics:

1. **Practice with Python examples** above
2. **Implement FK/IK** for different robot configurations
3. **Start Week 6 lab**: See [Week 6 Lab](labs/week-06.md)
4. **Work on homework problems** using these formulas

---

<div align="center">

**Ready to solve kinematics problems? Let's start the Week 6 lab! 📐**

[🐍 Python Basics](python-basics.md){ .md-button }
[🤖 ROS Setup](ros-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
