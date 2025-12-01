# 4-DOF Robot Arm Forward Kinematics

This repository implements a **Forward Kinematics Solver** for a custom 4-link serial manipulator using Linear Algebra concepts.

## 🧮 Mathematical Approach
Instead of relying on geometric decomposition (Trigonometry), which becomes complex and error-prone in 3D space, this solver uses **Homogeneous Transformation Matrices** following the **Denavit-Hartenberg (DH)** convention.

### The Core Algorithm
The position of the end-effector is calculated by transforming the coordinate frame from the base to the tip through a chain of matrix multiplications:

$$T_{global} = T_{0}^{1} \times T_{1}^{2} \times T_{2}^{3} \times T_{3}^{4}$$

Each transformation matrix $T_i$ is a $4 \times 4$ matrix composed of:
1.  **Rotation ($R_{3 \times 3}$):** Defined by the joint angle $\theta_i$ and the link twist $\alpha_i$.
2.  **Translation ($P_{3 \times 1}$):** Defined by the link length $a_i$ and offset $d_i$.
3.  **Linear Map:** $\begin{bmatrix} R & P \\ 0 & 1 \end{bmatrix}$

### [cite_start]Addressing the Geometry [cite: 11, 12]
The problem states that "the axis of each joint is perpendicular to the previous joint."
This allows us to construct the DH parameter table with alternating twist angles ($\alpha$):
* **Link 1:** Twist $\alpha = 90^\circ$ (Projects motion out of plane)
* **Link 2:** Twist $\alpha = -90^\circ$ (Projects motion back into plane)
* ...and so on.

## ⚠️ Edge Cases & Assumptions
1.  **Singularities:** While FK does not suffer from mathematical singularities like Inverse Kinematics, physical singularities (gimbal lock) can occur if two joint axes align (e.g., $J2$ parallel to $J4$). The matrix approach handles this robustly without crashing.
2.  **Coordinate Systems:** Without a marked 3D frame in the diagram, I assumed the standard DH convention where the $X_i$ axis points along the link $L_i$.
