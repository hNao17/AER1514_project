How to Obtain Data from Orbbec Astra
========================================

Bringing up the depth stream from the camera can be done using: 
-------------------------------------------------------------------

`roslaunch turtlebot_bringup 3dsensor.launch` 

Alternatively, the following 2 packages can be installed from Orbbec: 

https://github.com/orbbec/ros_astra_camera 

https://github.com/orbbec/ros_astra_launch 

The depth streams can be initialized by running 

`roslaunch astra_launch astrapro.launch` 


I could only access the RGB stream by installing usb_cam: 
-------------------------------------------------------------------

https://github.com/bosch-ros-pkg/usb_cam 

I used the test launch file to bring up the video stream, but I had to modify line 3 to: 

`<param name="video_device" value="/dev/video1" />` 

Otherwise, it picks up the video from the laptop camera.
