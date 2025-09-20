# SLAM with Custom Robot in Custom World 🌍🤖  

This project demonstrates **Simultaneous Localization and Mapping (SLAM)** using a **custom-built mobile robot** in a **custom Gazebo world**.  
The robot performs **mapping and localization** in real-time while navigating through the environment.  

---

## 🔧 Features
- Custom robot model designed in **URDF/Xacro**  
- Custom simulation world created in **Gazebo**  
- SLAM implemented using **ROS 2 (Nav2 / SLAM Toolbox / GMapping)**  
- Real-time mapping and localization  
- Teleoperation-based exploration to build the map  
- Generated occupancy grid maps for navigation  

---

## 🛠️ Tech Stack
- **ROS 2 Humble** (Robot Operating System)  
- **Gazebo** (simulation environment)  
- **RViz 2** (visualization)  
- **SLAM Toolbox / GMapping** (SLAM algorithm)  
- **Teleop Twist Keyboard** (manual control for exploration)  

---

## ⚙️ Setup & Installation
1. Clone the workspace:
   ```bash
   git clone https://github.com/<your-username>/slam-custom-robot.git
   cd slam-custom-robot
   colcon build
   source install/setup.bash

   Launch the custom world with robot:

ros2 launch my_robot_bringup world.launch.py


Start SLAM:

ros2 launch my_robot_bringup slam.launch.py


Control the robot (teleop):

ros2 run teleop_twist_keyboard teleop_twist_keyboard


Save the map:

ros2 run nav2_map_server map_saver_cli -f my_map

