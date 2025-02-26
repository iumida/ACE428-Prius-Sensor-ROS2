c這是更新後的 **Sensor Launch 使用手冊**，加入了 **CAN 總線開啟指令**，並放置於 **四、使用說明** 中，以便用戶在啟動感測器之前先開啟 CAN 總線。

---

### **Sensor Launch 使用手冊**

## **一、簡介**

Sensor Launch 是用於自動駕駛系統的感測器整合啟動工具，支援 USB 相機、IMU、Velodyne LiDAR、CAN 總線以及 GNSS 等設備，可快速完成所有感測器節點的啟動與資料整合，提供便利的資料錄製與可視化功能。

## **二、安裝需求**

- **ROS 2 Humble**
- **Ubuntu 22.04**
- **USB Camera**
- **IMU** (例如：Xsens MTi)
- **Velodyne LiDAR**
- **CAN 總線接口**
- **GNSS 接收器**（例如：Novatel）
- **Autoware Vehicle Msgs**

## **三、安裝步驟**

1. 安裝 **ROS 2 Humble**：

   ```bash
   sudo apt install ros-humble-desktop
   ```

2. 安裝相依套件：

   ```bash
   sudo apt install ros-humble-novatel-oem7-driver ros-humble-velodyne ros-humble-usb-cam ros-humble-autoware-auto-msgs
   ```

3. 從 GitHub 下載 **Sensor Launch** 套件：

   ```bash
   git clone https://github.com/iumida/ACE428-Prius-Sensor-ROS2.git
   cd ACE428-Prius-Sensor-ROS2
   ```

4. 編譯 **Sensor Launch** 套件：

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

5. 安裝 IMU (Xsens MTi)：

   ```bash
   git clone https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git -b ros2
   cd Xsens_MTi_ROS_Driver_and_Ntrip_Client
   colcon build --symlink-install
   source install/setup.bash
   ```

## **四、使用說明**

### **1. 開啟 CAN 總線**

在啟動感測器之前，請先開啟 CAN 總線：

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### **2. 啟動所有感測器節點**

```bash
ros2 launch sensorlaunch sensors_launch.py
```

### **3. 可視化資料**

使用 **RViz2** 來顯示點雲、影像及 IMU 資料：

```bash
rviz2
```

### **4. 錄製資料**

提供資料錄製腳本，可自動生成 `.db3` bag 檔案（如 `1.db3`、`2.db3` ...）：

```bash
python3 record_ros2_bag.py
```

## **五、感測器話題**

- **USB 相機:**

  - /camera1/image\_raw/compressed
  - /camera2/image\_raw/compressed
  - /camera3/image\_raw/compressed
  - /camera4/image\_raw/compressed
  - /camera5/image\_raw/compressed
  - /camera6/image\_raw/compressed

- **IMU:**

  - `/imu/data`
  - `/imu/acceleration`
  - `/imu/angular_velocity`

- **LiDAR:**

  - `/velodyne_points`

- **CAN 總線:**

  - `/steering_can`
  - `/velocity_can`

- **GNSS:**

  - `/novatel/oem7/bestpos`

- **濾波 Euler 角:**

  - `/filter/euler`

- **Autoware 訊息:**

  - `/steering_can`
  - `/velocity_can`

---

這樣就完成了 **CAN 總線開啟指令的整合**，現在這份使用手冊更完整，能確保用戶在啟動感測器之前正確開啟 CAN 總線！ 🚗💨

