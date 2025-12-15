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

3. Run `hostname -I` to learn your IP adress and use it for the following (Use your IP address)
   `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address> -p ROS_TCP_PORT:=11000`

4. ROS2, IP address and port number should match in Unity as well. If all works you will see blue bilateral arrows on left top corner of game screen with your ROS2 IP displayed.

---
 
## 4. Controlling the Robot via ROS

Firstly in unity side, attach "RobotPositionController.cs", "SimpleRosControl.cs", and "SimplePositionPublisher.cs" codes to the robot game object. You can also add a red circle image game object to the end effector to highlight.

Secondly, use mouse_simple.py to publish your mouse position and ros_tcp_endpoint to connect to unity from terminal

I have created alias for both of them in .bashrc so it is easier to run 

alias unity_ros='ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=164.11.236.21 -p ROS_TCP_PORT:=11000'

alias unity_mouse='ros2 run unity_robot_control mouse_simple'

---

## 5. Touchscreen UI Adjustments

Run the python script red_circle_cursor.py directly (ne need for ros) to add the UI elements. Pressing "r" reveals the rectangle and pressing "S" reveals sine wave path. Pressin the same key makes them dissappear.

~/unity_ws$ python3 red_circle_cursor.py 

---
## 6. Control using Joystick
Run the python script joystick_control.py directly (ne need for ros) to control the mouse position using the left side of the joystick. Pressing LT or RT increases the speed of mouse during control

~/unity_ws$ python3 joystick_control.py
