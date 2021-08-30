# MixedRealityHRC
Repo containing Unity project and ROS packages for developing mixed reality human-robot collaborative scenarios.

## Getting Started

### Unity Steps
Make sure you are using a version of Unity 2020.2.2 or newer. 
1) Create a new Unity project, then copy-paste the content of this repo in the Asset folder;
2) Download the ROS-TCP Connector package from https://github.com/Unity-Technologies/ROS-TCP-Connector and add it to the Unity project, to enable connection to a ROS environment;
3) Download the URDF Importer package from https://github.com/Unity-Technologies/URDF-Importer and add it to the Unity project to import your robot's URDF model. This project already contains models for Baxter and Panda robots.
4) Within the Robotics tab under Unity's main manu, insert the IP of the machine running the ROS environment.

### ROS Steps
1) The ROS/baxter_hrc folder contains the ROS package needed for motion planning and control. Copy-paste it in your ROS ws;
2) Download the ROS-TCP-Endpoint package from https://github.com/Unity-Technologies/ROS-TCP-Endpoint and add it to your ROS ws, then compile via catkin_make;
3) Within the config folder of baxter_hrc, change the ROS_IP to the IP of the current machine, whereas under UNITY_IP specify the IP of the machine which runs the Unity simulation

## Running the Simulation

### On ROS
MoveIt is used for robot motion planning: make sure to install the <your_robot>_moveit_config 
