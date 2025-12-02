# UR-Digital-Twin

A Unity-based digital twin of the **UR10** manipulator, developed to support testing and refinement of a touchscreen teleoperation interface. The project integrates Unity, ROS, inverse kinematics, and custom UI controls tailored for capacitive displays.

---

## 📌 Overview

This repository includes five key components:

1. **Simulating the UR10 Manipulator in Unity**  
2. **Controlling the Manipulator through Unity Scripts**  
3. **ROS–Unity Communication Setup**  
4. **Controlling the Robot via ROS**  
5. **Touchscreen UI Adjustments**

---

## 1. Simulating the UR10 Manipulator

1. Open Unity → **Package Manager** → **My Assets** → **+ (Add package from Git URL)**  
Enter:  https://github.com/Preliy/Flange.git#upm


2. Import robot samples via:  
**Samples → Flange → Demo**

3. Remove all robots except **UR10**, add a table, adjust the camera angle, and apply a floor texture.

More information on the Flange package:  
https://github.com/Preliy/Flange

---

## 2. Controlling the Manipulator in Unity

Create a folder called `Scripts` inside the `Assets` directory and add the two scripts from this repository:

### **RobotPositionController.cs**
- Accepts Cartesian target coordinates for the end effector.  
- Computes joint angles via inverse kinematics and moves the manipulator accordingly.

### **KeyboardRobotTester.cs**
- Allows manual testing of the manipulator with keyboard controls:  
- **W/A/S/D** → planar movement  
- **Q/E** → vertical movement  
- Includes adjustable end-effector speed and workspace boundaries.

---

## 3. ROS–Unity Communication

Follow the documentation at:  
https://github.com/Unity-Technologies/ROS-TCP-Connector

1. Add the package in Unity using:
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

2. Complete the setup steps described in:  
https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md

---
 
## 4. Controlling the Robot via ROS

---

## 5. Touchscreen UI Adjustments
