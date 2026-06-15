# 2R Planar Robot Kinematics & Manipulability Analyzer

An interactive Python-based robotics application that calculates and visualizes the linear velocity manipulability ellipsoids for a two-degree-of-freedom (2R) planar robotic arm. This project maps a manipulator's joint-space velocity constraints into a Cartesian task space using Singular Value Decomposition (SVD) to evaluate mechanical dexterity, directional isotropy, and proximity to kinematic singularities.

---

##  Visual Features

The analyzer evaluates specified joint configurations and dynamically overlays a scaled manipulability ellipse right at the end-effector tip:

* **Isotropic Configurations:** Circular profiles indicating uniform velocity capabilities in all directions.
* **Singular Configurations:** Completely flattened ellipses indicating a lost degree of freedom, where the manipulator cannot exert velocity along the short axis.

---

##  Kinematics & Mathematical Foundation

This project implements core theory outlined in foundational robotic textbooks like *Modern Robotics* (Lynch & Park).

### 1. Forward Kinematics
Given link lengths $L_1, L_2$ and joint angles $\theta_1, \theta_2$, the Cartesian position $(x_2, y_2)$ of the end-effector relative to a fixed base frame $(x_0, y_0) = (0,0)$ is mapped via geometric link projections:

$$x_1 = L_1 \cos(\theta_1)$$
$$y_1 = L_1 \sin(\theta_1)$$
$$x_2 = x_1 + L_2 \cos(\theta_1 + \theta_2)$$
$$y_2 = y_1 + L_2 \sin(\theta_1 + \theta_2)$$

### 2. Velocity Mapping & The Jacobian Matrix
The linear velocity Jacobian matrix $J_v \in \mathbb{R}^{2 \times 2}$ maps joint angular velocities $\dot{\theta} = \begin{bmatrix} \dot{\theta}_1 & \dot{\theta}_2 \end{bmatrix}^T$ to task-space linear velocities $V = \begin{bmatrix} \dot{x}_2 & \dot{y}_2 \end{bmatrix}^T$ such that:

$$V = J_v(\theta)\dot{\theta}$$

By differentiating the forward kinematics equations, the analytical linear velocity Jacobian is derived as:

$$J_v(\theta) = \begin{bmatrix} 
-L_1 \sin(\theta_1) - L_2 \sin(\theta_1 + \theta_2) & -L_2 \sin(\theta_1 + \theta_2) \\ 
L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) & L_2 \cos(\theta_1 + \theta_2) 
\end{bmatrix}$$

### 3. Singular Value Decomposition (SVD) & Ellipsoid Geometry
Assuming a unit sphere constraint on joint velocities ($\|\dot{\theta}\|^2 \le 1$), the mapping creates an ellipsoid in the velocity task-space. This project utilizes SVD to break down the Jacobian matrix:

$$J_v = U \Sigma V^T$$

* **Eigenvectors ($U$):** The columns of the orthogonal matrix $U$ define the rotation matrix of the ellipse axes relative to the base frame. The major axis angle is computed via:
  $$\phi = \operatorname{atan2}(U_{1,0}, U_{0,0})$$
* **Singular Values ($\Sigma = \operatorname{diag}(\sigma_1, \sigma_2)$):** The singular values ($\sigma_{\max}, \sigma_{\min}$) correspond directly to the lengths of the semi-major and semi-minor axes of the manipulability ellipsoid.

---

##  Quantitative Dexterity Metrics

The script calculates and outputs three primary performance indicators to evaluate the robot's posture:

| Metric | Mathematical Form | Description |
| :--- | :--- | :--- |
| **Yoshikawa's Manipulability Measure ($\mu_1$)** | $\sqrt{\det(J_v J_v^T)} = \sigma_1 \sigma_2$ | Proportional to the volume of the ellipsoid. High values denote massive multi-directional mobility; a value of $0$ indicates a kinematic singularity. |
| **Condition Number ($\mu_2$)** | $\frac{\sigma_{\max}}{\sigma_{\min}}$ | Measures directional fluidity (isotropy). A value of $1.0$ represents a perfect sphere (uniform movement ease). Higher numbers mean poor directional movement. |
| **Distance to Singularity ($\mu_3$)** | $\sigma_{\min}$ | Monitors structural limits by tracking the magnitude of the weakest operational vector. |

---

##  Installation & Execution

### 1. Prerequisites
Ensure you have Python 3.8+ installed along with standard environment managers.

### 2. Clone the Repository
```bash
git clone [https://github.com/23ME10007/2R-Planar-Robot-Manipulability.git](https://github.com/23ME10007/2R-Planar-Robot-Manipulability.git)
cd 2R-Planar-Robot-Manipulability
