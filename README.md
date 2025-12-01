
# 4-DOF Robot Arm Forward Kinematics

This repository implements a **Forward Kinematics Solver** for a 4-link serial manipulator using Homogeneous Transformation Matrices (Denavit-Hartenberg Convention).

## 🚀 How to Run the Code
(Follow these steps during the live interview)

### 1. Setup
Clone the repository and install the dependencies (NumPy).
```bash
git clone [https://github.com/Karthikeya-g/robot-arm-fk.git](https://github.com/Karthikeya-g/robot-arm-fk.git)
cd robot-arm-fk
pip install -r requirements.txt
````

### 2\. Run Verification

I have included a verification script that checks the kinematics against 4 known geometric edge cases (Zero Pose, 90° Base, Cobra Pose, and Corkscrew).

**Run the command:**

```bash
python verify_fk.py
```

**Expected Output:**

```text
✅ PASS | Zero Pose (Straight Out)
✅ PASS | 90 Deg Base Rotation
✅ PASS | Cobra Pose (Bend Up)
✅ PASS | Corkscrew (3D Turn)
```

-----

## 💻 Usage Example

You can import the solver as a module in your own Python scripts:

```python
from src import ForwardKinematics, JointState

# Initialize robot with link length L = 1.0m
fk = ForwardKinematics(link_length=1.0)

# Define joint angles (Degrees)
joints = JointState(j1=0, j2=90, j3=90, j4=0)

# Calculate Position
x, y, z = fk.solve(joints)
print(f"End Effector: ({x}, {y}, {z})")
```

-----

## 🧮 Mathematical Approach

Instead of relying on geometric decomposition (Trigonometry), which is error-prone in 3D, this solver uses **Linear Algebra** via **Homogeneous Transformation Matrices**.

### The Algorithm

The end-effector position is calculated by chaining frame transformations from the base to the tip:

$$T_{total} = T_{0}^{1} \times T_{1}^{2} \times T_{2}^{3} \times T_{3}^{4}$$

Each $T_i$ is a $4 \times 4$ matrix derived from the **Modified Denavit-Hartenberg (DH)** parameters:

1.  **Rotation ($R_{3 \times 3}$):** Defined by joint angle $\theta$ and link twist $\alpha$.
2.  **Translation ($P_{3 \times 1}$):** Defined by link length $a$ and offset $d$.

### Geometric Assumptions

Based on the problem statement "the axis of each joint is perpendicular to the previous joint":

  * **Twist ($\alpha$):** Alternates between $90^\circ$ and $-90^\circ$ to represent the orthogonal axes.
  * **Coordinate Frame:** A Positive Joint 2 rotation moves the arm **Up** (+Z), and a subsequent Positive Joint 3 rotation moves it **Left** (+Y). This coordinate sign convention was verified via the "Corkscrew" test case.

-----
