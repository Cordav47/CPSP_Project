# Drone Simulation Project

This project demonstrates a PX4-based drone simulation integrated with ROS 2, Gazebo, and QGroundControl, featuring:

- **PX4 SITL** for realistic drone flight dynamics  
- **Gazebo** for 3D simulation  
- **QGroundControl** for monitoring and mission management  
- **Micro XRCE-DDS Agent** (micro-RTPS bridge) for ROS 2 and PX4 communication  
- **ROS 2 Package** with:  
  - Two image publisher nodes (`risk_image_publisher`, `standard_image_publisher`)  
  - An image processing node (`image_processor`)  
  - A goal producer node (`goal_producer`)  
  - A drone control node (`drone_control`)
  - The px4_msgs library (`px4_msg`) to communicate with the simulation
  - A custom goal message (`goal_msg`) to map pixel into goal

## Table of Contents

1. [Project Structure](#project-structure)  
2. [Prerequisites](#prerequisites)  
3. [Building the Simulation](#building-the-simulation)  
4. [Nodes Description](#nodes-description)  
5. [License](#license)

---

## Project Structure

. ├── PX4-Autopilot/ ├── ros2_ws/ │ ├── src/ │ │ └── drone_project/ │ │ ├── drone_project/ │ │ │ ├── init.py │ │ │ ├── risk_image_publisher.py │ │ │ ├── standard_image_publisher.py │ │ │ ├── image_processor.py │ │ │ ├── goal_producer.py │ │ │ ├── drone_control.py │ │ │ ├── My_CV_library.py │ │ ├── launch/ │ │ │ └── drone_launch.py │ │ ├── package.xml │ │ ├── setup.py ├── QGroundControl/ └── ...


---

## Prerequisites

- **ROS 2** (Humble or later)  
- **PX4-Autopilot** (latest version)  
- **Gazebo** (installed with PX4 dependencies)  
- **QGroundControl**  
- **Micro XRCE-DDS Agent**  
- **Python 3** with ROS 2 packages:
  - `rclpy`, `std_msgs`, `sensor_msgs`, `cv_bridge`, `px4_msgs`
- **OpenCV**  
- **Ament build tools**: `colcon`

For a complete guide of the setup refers to https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup

---

## Building the Simulation

1. **Build PX4-Autopilot**:

   ```shell
   cd PX4-Autopilot
   make px4_sitl gazebo

2. **Establish ros2-px4 connection**:

   ```shell
   cd folder/Micro-XRCE-DDS-Agent
   MicroXRCEAgent udp4 -p 8888


3. **Launch QGroundControl**:

   ```shell
   cd folder
   ./AppQGroundControl.Image

4. **Launch the project**:

   ```shell
   cd folder/project
   colcon build
   #source ros2 and gazebo if they aren't
   . install/setup.bash
   ros2 launch CPSP_Project Project.launch.py


