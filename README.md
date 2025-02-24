# Sensor Launch 使用手冊

## 一、簡介

Sensor Launch是用於自動駕駛系統的感測器整合啟動工具，支援USB相機、IMU、Velodyne LiDAR、CAN總線以及GNSS等設備，可快速完成所有感測器節點的啟動與資料整合，提供便利的資料錄製與可視化功能。

## 二、安裝需求

- ROS 2 Humble
- Ubuntu 22.04
- USB Camera
- IMU (例如：Xsens MTi)
- Velodyne LiDAR
- CAN 總線接口
- GNSS 接收器（例如：Novatel）
- Autoware Vehicle Msgs

## 三、安裝步驟

1. 安裝ROS 2 Humble：

   ```bash
   sudo apt install ros-humble-desktop
   ```

2. 安裝相依套件：

   ```bash
   sudo apt install ros-humble-velodyne ros-humble-usb-cam ros-humble-autoware-auto-msgs
   ```

3. 從GitHub下載Sensor Launch套件：

   ```bash
   git clone https://github.com/iumida/ACE428-Prius-Sensor-ROS2.git
   cd ACE428-Prius-Sensor-ROS2
   ```

4. 編譯Sensor Launch套件：

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

5. IMU手動安裝（Xsens MTi）：

   ```bash
   git clone https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git -b ros2
   cd Xsens_MTi_ROS_Driver_and_Ntrip_Client
   colcon build --symlink-install
   source install/setup.bash
   ```

## 四、使用說明

### 啟動所有感測器節點：

```bash
ros2 launch sensorlaunch sensors_launch.py
```

### 可視化資料：

使用RViz2可視化點雲、影像及IMU資料：

```bash
rviz2
```

### 錄製資料：

提供資料錄製腳本，可自動生成bag檔案（如1.db3、2.db3...）：

```bash
python3 record_ros2_bag.py
```

## 五、感測器話題

- USB相機:

  - `/camera1~6/image_raw/compressed`

- IMU:

  - `/imu/data`
  - `/imu/acceleration`
  - `/imu/angular_velocity`

- LiDAR:

  - `/velodyne_points`

- CAN:

  - `/steering_can`
  - `/velocity_can`

- GNSS:

  - `/novatel/oem7/bestpos`

- 濾波Euler角:

  - `/filter/euler`

- Autoware訊息:

  - `/steering_can`
  - `/velocity_can`



