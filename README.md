# ACE428-Prius-Sensor-ROS2

source ~/sensorlaunch/install/setup.bash
ros2 launch sensorlaunch sensors_launch.py
//開感測器

ros2 topic echo /velodyne_points 
//光達資訊

ros2 topic hz /velodyne_points 
//光達頻率