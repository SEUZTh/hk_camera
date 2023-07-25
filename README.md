# hk_camera

Tested on Ubuntu 20.04 with ROS2 Foxy.

This code repository is based on [HIKROBOT-MVS-CAMERA-ROS](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS), and has been upgraded from ROS1 to ROS2.

You can choose to publish both raw image messages and compressed image messages:
```bash
ros2 run hk_camera hk_camera
ros2 run hk_camera hk_camera_compressed
```