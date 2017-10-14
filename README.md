# AER1514_project
Workspace for turtlebot software development for a QR code scavenger hunt. The turtlebot must autonomously navigate an indoor environment, recognizing and deciphering as many QR codes as possible within an allotted time, before returning and docking itself to its charging station. An overview of the system architecture is shown below:

![Architecture](https://github.com/hNao17/AER1514_project/blob/master/architecture.PNG)

Four high-level nodes were designed to achieve the mission objectives: 1) Navigation, 2) QR Detect, 3) Dock Detect, and 4) Docking. Each of these is managed by a Supervisor node. This project was developed relying extensively on existing packages in ROS. Custom functionalities have been implemented via C++ scripts, contained in the /walle/src/scripts/ folder. 


