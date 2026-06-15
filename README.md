# 2R Planar Robot Kinematics & Manipulability Analyzer

An interactive Python-based robotics application that calculates and visualizes the linear velocity manipulability ellipsoids for a two-degree-of-freedom (2R) planar robotic arm. This project maps a manipulator's joint-space velocity constraints into a Cartesian task space using Singular Value Decomposition (SVD) to evaluate mechanical dexterity, directional isotropy, and proximity to kinematic singularities.

---

## 🚀 Visual Features

The analyzer evaluates specified joint configurations and dynamically overlays a scaled manipulability ellipse right at the end-effector tip:

* **Isotropic Configurations:** Circular profiles indicating uniform velocity capabilities in all directions.
* **Singular Configurations:** Completely flattened ellipses indicating a lost degree of freedom, where the manipulator cannot exert velocity along the short axis.

---

## 🧠 Kinematics & Mathematical Foundation

This project implements core theory outlined in foundational robotic textbooks like *Modern Robotics* (Lynch & Park).

### 1. Forward Kinematics
Given link lengths L1, L2 and joint angles theta1, theta2, the Cartesian position (x2, y2) of the end-effector relative to a fixed base frame (x0, y0) = (0,0) is mapped via geometric link projections:

$$x_1 = L_1 \cos(\theta_1)$$
$$y_1 = L_1 \sin(\theta_1)$$
$$x_2 = x_1 + L_2 \cos(\theta_1 + \theta_2)$$
$$y_2 = y_1 + L_2 \sin(\theta_1 + \theta_2)$$

### 2. Velocity Mapping & The Jacobian Matrix
The linear velocity Jacobian matrix ($J_v$) is a 2x2 matrix that maps joint angular velocities to task-space linear velocities. 

* The joint velocity vector is defined as: $\dot{\theta} = [\dot{\theta}_1, \dot{\theta}_2]^T$
* The task-space linear velocity vector is defined as: $V = [\dot{x}_2, \dot{y}_2]^T$

These spaces map to each other via the linear transformation:

$$V = J_v(\theta)\dot{\theta}$$

By differentiating the forward kinematics equations, the analytical linear velocity Jacobian is derived as:

$$J_v(\theta) = \begin{bmatrix} 
-L_1 \sin(\theta_1) - L_2 \sin(\theta_1 + \theta_2) & -L_2 \sin(\theta_1 + \theta_2) \\ 
L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) & L_2 \cos(\theta_1 + \theta_2) 
\end{bmatrix}$$

### 3. Singular Value Decomposition (SVD) & Ellipsoid Geometry
Assuming a unit sphere constraint on joint velocities, where the sum of squared joint velocities is less than or equal to 1, the mapping creates an ellipsoid in the velocity task-space. This project utilizes SVD to break down the Jacobian matrix:

$$J_v = U \Sigma V^T$$

* **Eigenvectors (U):** The columns of the orthogonal matrix U define the rotation matrix of the ellipse axes relative to the base frame. The major axis angle is computed via:
  $$\phi = \text{atan2}(U_{1,0}, U_{0,0})$$
* **Singular Values:** The singular values (sigma1, sigma2) correspond directly to the lengths of the semi-major and semi-minor axes of the manipulability ellipsoid.

---

## 📊 Quantitative Dexterity Metrics

The script calculates and outputs three primary performance indicators to evaluate the robot's posture:

### 1. Yoshikawa's Manipulability Measure
* **Mathematical Form:** $$\mu_1 = \sqrt{\text{det}(J_v J_v^T)} = \sigma_1 \sigma_2$$
* **Description:** Proportional to the volume of the ellipsoid. High values denote massive multi-directional mobility; a value of 0 indicates a kinematic singularity.

### 2. Condition Number
* **Mathematical Form:** $$\mu_2 = \frac{\sigma_1}{\sigma_2}$$
* **Description:** Measures directional fluidity (isotropy). A value of 1.0 represents a perfect sphere (uniform movement ease). Higher numbers mean poor directional movement.

### 3. Distance to Singularity
* **Mathematical Form:** $$\mu_3 = \sigma_2$$
* **Description:** Monitors structural limits by tracking the magnitude of the weakest operational vector (the smallest singular value).

---

## 🛠️ Installation & Execution

### 1. Prerequisites
Ensure you have Python 3.8+ installed along with standard environment managers.

### 2. Clone the Repository
```bash
git clone [https://github.com/23ME10007/2R-Planar-Robot-Manipulability.git](https://github.com/23ME10007/2R-Planar-Robot-Manipulability.git)
cd 2R-Planar-Robot-Manipulability
