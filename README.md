# Alex to the Rescue – CG2111A Project

## Overview
This project was developed as part of **CG2111A – Engineering Principles and Practice II** at NUS.  
**Alex** is a tele-operated robot designed for simulated disaster-response scenarios. It navigates remote terrain, maps the environment, and interacts with color-coded astronauts, performing both rescue and med-pak delivery tasks.

## Features
- **Secure Teleoperation** – TLS-based communication between laptop and Raspberry Pi  
- **Mapping & Navigation** – 360° RPLIDAR with ROS Hector SLAM for real-time environment mapping  
- **Dual Claw System** – Front claw for astronaut rescue and rear claw for med-pak delivery  
- **Color Detection** – TCS3200 color sensor to differentiate between red (rescue) and green (supply drop) targets  
- **Ultrasonic Sensing** – Collision avoidance and astronaut alignment  
- **Tick-based Motion Control** – Wheel encoders for accurate distance and turning  

## Tech & Tools
- **Hardware:** Raspberry Pi 4, Arduino, RPLIDAR, Pi Camera, Ultrasonic Sensors, TCS3200 Color Sensor, DC motors with encoders, Servo motors  
- **Software:** ROS, Hector SLAM, TLS networking, Arduino C/C++, Raspberry Pi Python server  
- **Skills Applied:** Embedded systems, secure communication, robotics control, real-time mapping, teamwork  

## Setup
1. Power the Raspberry Pi and Arduino subsystems (separate modular battery packs).  
2. Start the TLS server on the Raspberry Pi and connect the laptop client.  
3. Launch ROS mapping with:  
   ```bash
   roslaunch rplidar_ros view_slam.launch
- Run `AlexCameraStreamServer.py` on the Raspberry Pi to enable the video feed.  
- Use the laptop interface to send commands (forward, reverse, turn, stop, open/close claws, detect color, read distances).  

## Team Members
- **Sim Chin Kai Alson** – Software (TLS + ROS)  
- **Lim Swee How Gabriel** – Hardware (Circuitry)  
- **Shah Kushal Hitesh** – Firmware (Sensors + RPi)  
- **Vansh Puri** – Software (Arduino Code)  

## Reflection
Building Alex required integrating multiple subsystems into a cohesive robotic platform.  
We overcame challenges in claw design, color sensor placement, and consistent TLS communication.  
The final system successfully mapped the maze, detected astronauts, and performed rescue tasks,  
demonstrating practical disaster-response robotics.
