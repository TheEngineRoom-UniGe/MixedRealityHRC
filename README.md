# MixedRealityHRC
Repo containing Unity project and ROS packages for developing mixed reality human-robot collaborative scenarios.

## Getting Started
The instructions provided here assume a configuration in which 2 machines, one running the ROS environment, whereas the other runs the Unity simulation, are connected to the same local network. 

### Unity Steps
Make sure you are using a version of Unity 2020.2.2 or newer. 
1) Create a new Unity project, then copy-paste the content of this repo in the Asset folder;
2) Download the ROS-TCP Connector package from https://github.com/Unity-Technologies/ROS-TCP-Connector and add it to the Unity project, to enable connection to a ROS environment;
3) Download the URDF Importer package from https://github.com/Unity-Technologies/URDF-Importer and add it to the Unity project to import your robot's URDF model. This project already contains models for Baxter and Panda robots.
4) Within the Robotics tab under Unity's main manu, insert the IP of the machine running the ROS environment.

### ROS Steps
1) The ROS/baxter_hrc folder contains the ROS package needed for motion planning and control. Copy-paste it in your ROS ws;
2) Download the ROS-TCP-Endpoint package from https://github.com/Unity-Technologies/ROS-TCP-Endpoint and add it to your ROS ws, then compile via catkin_make;
3) Within the config folder of baxter_hrc, change the ROS_IP to the IP of the current machine, whereas under UNITY_IP specify the IP of the machine which runs the Unity simulation.

## Running the Simulation

### On ROS
1) MoveIt is used for robot motion planning: make sure to install the <your_robot>_moveit_config from https://moveit.ros.org/robots/;
2) Launch the motion planner node and the server endpoint which enables communication with Unity via the launch/main.launch file.

### On Unity

1) Start the simulation;
2) Ensure the handshake between ROS and Unity is established; Communication issues may be related to Windows Defender Firewall (try deactivating it) or closed ports which need to be opened manually).

## Deploy on HoloLens device

1) Within the config folder in the ROS package, change the UNITY_IP to that of the HoloLens device;
2) Compile the Unity application with UWP build settings. Ensure Internet capabilities and spatial perception are ON in the player settings. Finally, deploy on device and run

