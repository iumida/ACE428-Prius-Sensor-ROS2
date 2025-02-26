### **Sensor Launch 使用手冊**

## **一、簡介**

Sensor Launch 是用於自動駕駛系統的感測器整合啟動工具，支援 USB 相機、IMU、Velodyne LiDAR、CAN 總線以及 GNSS 等設備，可快速完成所有感測器節點的啟動與資料整合，提供便利的資料錄製與可視化功能。

## **二、安裝需求**

### **系統版本要求**
- **Ubuntu 22.04**
- **ROS 2 Humble**

### **限定硬體**

- **1920x1280 USB oTo CAM X6**（USB 連線）
- **IMU: XSENS MTI 630R**（USB 連線）
- **NovAtel FlexPak6 GPS**（USB 連線）
- **CAN 總線**（USB 連線）
- **VLP16 LiDAR**（網路連線）

## **三、安裝步驟**

1. 設定 ROS 2 安裝來源：

   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt upgrade
   ```

2. 安裝 **ROS 2 Humble**：

   ```bash
   sudo apt install ros-humble-desktop
   ```

3. 安裝相依套件：

   ```bash
   sudo apt install ros-humble-novatel-oem7-driver ros-humble-velodyne ros-humble-usb-cam ros-humble-autoware-auto-msgs
   ```

4. 從 GitHub 下載 **Sensor Launch** 套件：

   ```bash
   git clone https://github.com/iumida/ACE428-Prius-Sensor-ROS2.git
   cd ACE428-Prius-Sensor-ROS2
   ```

5. 編譯 **Sensor Launch** 套件：

   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

6. 安裝 IMU (Xsens MTi)：

   ```bash
   git clone https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git -b ros2
   cd Xsens_MTi_ROS_Driver_and_Ntrip_Client
   colcon build --symlink-install
   source install/setup.bash
   ```

## **四、使用說明**

### **ROS 2 Bag 解析與資料儲存腳本**

此腳本用於解析 ROS 2 bag 檔案，並提取感測器數據（相機影像、LiDAR 點雲、IMU、CAN 訊號、GPS 等）並儲存為對應的格式。

**功能概述：**
- 讀取 ROS 2 bag 檔案，解析所有指定話題。
- 以時間同步的方式篩選有效數據點。
- 儲存 IMU、CAN、GPS、LiDAR、影像數據到指定的資料夾。
- 影像數據儲存為 JPG，LiDAR 數據儲存為 PCD。

**執行方式：**
```bash
python3 unbagall.py
```
**輸入與輸出格式：**
- **輸入**：ROS 2 bag 檔案資料夾。
- **輸出**：分類儲存的感測器數據（影像、LiDAR、IMU、GPS 等）。

### **ROS 2 指令介紹**

- `ros2 topic echo <topic_name>`：顯示指定話題的即時數據。
- `ros2 topic hz <topic_name>`：監測話題的發送頻率。
- `ros2 topic list`：列出所有可用的話題。
- `ros2 bag info <bag_file>`：顯示 ROS 2 Bag 檔案的資訊。
- `ros2 bag play <bag_file>`：播放 ROS 2 Bag 錄製的數據。

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
  - /camera1/image_raw/compressed
  - /camera2/image_raw/compressed
  - /camera3/image_raw/compressed
  - /camera4/image_raw/compressed
  - /camera5/image_raw/compressed
  - /camera6/image_raw/compressed

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

