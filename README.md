# GoMavSimulator-ros2-demo
A demo package for [GoMavSimulator](https://github.com/timetravelCat/GoMavSimulator)

## Getting Started
### Build GoMavSimulator-ros2-demo package. 
- No extra dependency required. (ROS2 development environments are required.)
```bash
colcon build --merge-install
. install/setup.bash # call install/setup.bat 
ros2 run GoMavSimulator-ros2-demo VehicleSensorSubscriber
```

- [TODO Attach Video]

### Install GoMavSimulator
- Download latest [release](https://github.com/timetravelCat/GoMavSimulator/releases)
- Run `GoMavSimulator` (run as `administrator` if `windows`, OpenSSL required in system. see [GoMavSimulator docs](https://github.com/timetravelCat/GoMavSimulator/blob/main/docs/GettingStarted.md))

### Add vehicle for simulation
- If GoMavSimulator runs, press `ESC` and select `Simulator` tab.
- Fill `VEHICLE NAME` as `vehicle`
- Select `POSE SOURCE` as `USER`
- Press `ADD`, vehicle is created with multicopter model.

### Subscribe vehicle pose 
- With `vehicle` selected, click `ADVANCED CONFIGURATION`
- Toggle `PUBLISH VEHICLE POSE` to be enabled. 
    - topic_name : `/vehicle_pose` topic.
    - topic_type : geometry_msgs/msg/PoseStamped

### Subscribe range sensor
- Select `vehicle` in Simulator Settings in GoMavSimulator
- Add RANGE(ToF) sensor named with `range`, [0, 0, 0] rotation and position
    - Zero rotation & position make range sensor face front.
    - Set maximum distance in advanced configuration.
    - topic name : `vehicle/range`
    - topic type : sensor_msgs/msg/Range 
- VehicleSensorSubscriber node will subscribe `vehicle/range` and publish `vehicle/range_result` as geometry_msgs/msg/PointStamped with global reference frame, compensated by vehicle pose. 

### Subscribe lidar sensor
- Select `vehicle` in Simulator Settings in GoMavSimulator
- Add Lidar sensor named with `lidar`
    - See lidar sensor section of  [Settings](!https://github.com/timetravelCat/GoMavSimulator/blob/main/docs/Settings.md) for advanced configuration.
    - topic name : `vehicle/lidar`
    - topic type : sensor_msgs/msg/PointCloud2
- VehicleSensorSubscriber node will subscribe `vehicle/lidar` and publish `vehicle/lidar_result` as sensor_msgs/msg/PointCloud2 with global reference frame, compensated by vehicle pose. 

### Subscribe image sensor
- Select `vehicle` in Simulator Settings in GoMavSimulator
- Add RGB image sensor named `rgb`
    - topic name : `vehicle/rgb`
    - topic type : sensor_msgs/msg/Image 

### Subscribe depth image sensor
- Select `vehicle` in Simulator Settings in GoMavSimulator
- Add Depth image sensor named `depth`
    - topic name : `vehicle/depth`
    - topic type : sensor_msgs/msg/Image 
- VehicleSensorSubscriber node will subscribe `vehicle/depth` and publish `vehicle/depth_result` as sensor_msgs/msg/PointCloud2 with global reference frame, compensated by vehicle pose. 