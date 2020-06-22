# azure_kinect_ros
Docker for utilize azure kinect and body track on ros

# Requirement
* Azure Kinect
* Nvidia Docker2

# Environment 
* microsoft/Azure Kinect Sensor SDK Version : [1.3](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/release/1.3.x)
* microsoft/Azure Kinect Body Tracking SDK Version : 1.0
* microsoft/[Azure Kinect Samples](https://github.com/microsoft/Azure-Kinect-Samples) : Enable (Build ◎)
* microsoft/[Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) : Enable (Build ◎)
    * Body Track : Enable (Build ◎)
* Docker Base Image : nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04
* ROS Distribution : melodic 

# Usage
## Launch Docker
1. Docker Build
    ```
    ./build
    ```
2. Docker Run
    ```
    ./run
    ```

##  Azure Kinect SDK
```
$  cd /home/Azure-Kinect-Sensor-SDK/build/bin
$ ./k4aviewer 
```

## Body Track SDK
```
$  cd /home/Azure-Kinect-Samples/build/bin
$ ./simple_3d_viewer
```

## ROS without Body Track
```
$  roslaunch azure_kinect_ros_driver driver.launch 
```

## ROS with Body Track
```
$  roslaunch azure_kinect_ros_driver bodyTrack.launch 
```

## ROS with user's launch
　Please put user's launch in `user_launch` folder.

# Lisence
　[MIT](https://github.com/chakio/azure_kinect_ros/blob/master/LICENCE)

# Author
　[chakio](https://github.com/chakio)