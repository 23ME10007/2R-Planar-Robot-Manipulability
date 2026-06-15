#!/usr/bin/env python
# coding: utf-8

# ## 1. Exporting libraries

# In[4]:


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse 


# In[8]:


def get_joint_positions(L1, L2, th1, th2):
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_title(f"2R Robot Configuration (L1={L1}, L2={L2})")
    ax.set_xlabel("X Space")
    ax.set_ylabel("Y Space")
    
    # Correctly convert degrees to radians
    th1_rad = np.radians(th1)
    th2_rad = np.radians(th2)

    # Forward Kinematics calculations (using the _rad variables!)
    x0, y0 = 0.0, 0.0
    x1 = L1 * np.cos(th1_rad)
    y1 = L1 * np.sin(th1_rad)
    x2 = x1 + L2 * np.cos(th1_rad + th2_rad)
    y2 = y1 + L2 * np.sin(th1_rad + th2_rad)

    # Plot the arm
    ax.plot([x0, x1, x2], [y0, y1, y2], marker='o', linewidth=3, color='blue', label=f"({th1}°, {th2}°)")
    ax.legend()
    
    
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    plt.show()

# Run the function with your test configuration
get_joint_positions(1, 1, 60, 60)


# In[ ]:





# ## Ellipsoid 

# In[ ]:





# In[13]:


def compute_jacobian(L1, L2, th1, th2):
    # Correctly convert degrees to radians
    th1_rad = np.radians(th1)
    th2_rad = np.radians(th2)
    #Setup the figure and axis inside the function or pass it in
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_title(f"Configuration & Manipulability Ellipse")

    #Forward Kinematics calculations 
    x0, y0 = 0.0, 0.0
    x1 = L1 * np.cos(th1_rad)
    y1 = L1 * np.sin(th1_rad)
    x2 = x1 + L2 * np.cos(th1_rad + th2_rad)
    y2 = y1 + L2 * np.sin(th1_rad + th2_rad)

    Jv = np.array([[(-L1 * np.sin(th1_rad))+(-L2 * np.sin(th1_rad + th2_rad)), -L2 * np.sin(th1_rad+th2_rad)],
                   [(L1 * np.cos(th1_rad))+(L2 * np.cos(th1_rad + th2_rad)), L2 * np.cos(th1_rad+th2_rad)]])
    print(Jv)

    
    A = Jv @ Jv.T                        #A matrix for volume calculation
    
    U, S, Vt = np.linalg.svd(Jv)         #For assigning values 
    sigma_max = S[0]
    sigma_min = S[1]
    
    #calculating mus
    mu1 = np.sqrt(np.linalg.det(A))       # Yoshikawa's measure (Volume)
    mu2 = S[0] / S[1]                     # Condition number (Isotropy indicator; closer to 1 is better)
    mu3 = S[1]

    #plotting the ellipse now
    # Angle of the major axis relative to the x-axis
    ellipse_angle = np.degrees(np.arctan2(U[1, 0], U[0, 0]))

    ellipse_scale = 0.3                  #choosing appropriate scale 
    # Creating matplotlib ellipse patch centered at end-effector (x2, y2): 
    # Width/Height represent full diameters, so we multiply singular values by 2
    ell = Ellipse(
    xy=(x2, y2), 
    width=2 * sigma_max * ellipse_scale, 
    height=2 * sigma_min * ellipse_scale, 
    angle=ellipse_angle, 
    edgecolor='black', 
    facecolor='none', 
    linestyle='--', 
    linewidth=1.5
            )
    ax.add_patch(ell)


# In[15]:


compute_jacobian(1,1,160,90)


# In[17]:


compute_jacobian(1,1,-10,10)

