import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from bisect import bisect_left
import numpy as np
import cv2

# 用戶輸入
bag_folder = input("請輸入 ROS 2 bag 資料夾路徑: ")
output_parent_folder = input("請輸入資料保存位置 (例如 /home/user/output): ")

# 創建輸出資料夾
os.makedirs(output_parent_folder, exist_ok=True)

# 設定 topics
all_topics = [
    '/velodyne_points', '/camera1/image_raw/compressed', '/camera2/image_raw/compressed',
    '/camera3/image_raw/compressed', '/camera4/image_raw/compressed', '/camera5/image_raw/compressed',
    '/camera6/image_raw/compressed', '/steering_can', '/velocity_can', '/novatel/oem7/bestpos',
    '/imu/data', '/filter/euler'
]

# 初始化 reader
storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr", output_serialization_format="cdr"
)
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# 提取 topic_types
topics = reader.get_all_topics_and_types()
topic_types = {topic.name: topic.type for topic in topics}

# 儲存所有 topic 的時間戳
all_timestamps = {topic: [] for topic in all_topics}

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    timestamp_sec = timestamp / 1e9
    if topic in all_topics:
        all_timestamps[topic].append(timestamp_sec)

# 找到最少資料的 topic 作為時間基準
min_topic = min(all_timestamps, key=lambda k: len(all_timestamps[k]))
base_times = all_timestamps[min_topic]

print(f"基準 topic 為 {min_topic}, 資料數量: {len(base_times)}")

# 比對時間並取得有效時間戳
valid_times = []
time_tolerance = 0.15

for base_time in base_times:
    valid = True
    for topic, timestamps in all_timestamps.items():
        idx = bisect_left(timestamps, base_time)
        if idx >= len(timestamps) or abs(timestamps[idx] - base_time) > time_tolerance:
            valid = False
            break
    if valid:
        valid_times.append(base_time)

print(f"有效資料點數量: {len(valid_times)}")

# 處理並儲存資料
reader.open(storage_options, converter_options)

topic_data = {topic: [] for topic in all_topics}
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    timestamp_sec = timestamp / 1e9
    if topic in all_topics:
        topic_data[topic].append((timestamp_sec, data))

for lidar_time in valid_times:
    for topic, messages in topic_data.items():
        idx_msg = bisect_left([msg[0] for msg in messages], lidar_time)
        deserialized_msg = deserialize_message(messages[idx_msg][1], get_message(topic_types[topic]))

        if topic == '/novatel/oem7/bestpos':
            for folder, value in zip(['Latitude', 'Longitude'], [deserialized_msg.lat, deserialized_msg.lon]):
                path = os.path.join(output_parent_folder, folder)
                os.makedirs(path, exist_ok=True)
                with open(f"{path}/{lidar_time:.6f}.txt", 'w') as f:
                    f.write(str(value))

        elif topic in ['/steering_can', '/velocity_can']:
            path = os.path.join(output_parent_folder, topic.strip('/'))
            os.makedirs(path, exist_ok=True)
            value = deserialized_msg.steering_tire_angle if topic == '/steering_can' else deserialized_msg.longitudinal_velocity
            with open(f"{path}/{lidar_time:.6f}.txt", 'w') as f:
                f.write(str(value))

        elif topic == '/imu/data':
            imu_data = {
                'imu_roll_acc': deserialized_msg.angular_velocity.x,
                'imu_pitch_acc': deserialized_msg.angular_velocity.y,
                'imu_yaw_acc': deserialized_msg.angular_velocity.z,
                'imu_acc_x': deserialized_msg.linear_acceleration.x,
                'imu_acc_y': deserialized_msg.linear_acceleration.y,
                'imu_acc_z': deserialized_msg.linear_acceleration.z
            }
            for folder, value in imu_data.items():
                path = os.path.join(output_parent_folder, folder)
                os.makedirs(path, exist_ok=True)
                with open(f"{path}/{lidar_time:.6f}.txt", 'w') as f:
                    f.write(str(value))

        elif topic == '/filter/euler':
            euler_data = ['imu_roll', 'imu_pitch', 'imu_yaw']
            values = [deserialized_msg.vector.x, deserialized_msg.vector.y, deserialized_msg.vector.z]
            for folder, value in zip(euler_data, values):
                path = os.path.join(output_parent_folder, folder)
                os.makedirs(path, exist_ok=True)
                with open(f"{path}/{lidar_time:.6f}.txt", 'w') as f:
                    f.write(str(value))

        elif topic == '/velodyne_points':
            points = np.frombuffer(deserialized_msg.data, dtype=np.float32).reshape(-1, 4)
            path = os.path.join(output_parent_folder, 'pcd_files')
            os.makedirs(path, exist_ok=True)
            with open(f"{path}/{lidar_time:.6f}.pcd", 'w') as f:
                f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
                f.write(f'WIDTH {points.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {points.shape[0]}\nDATA ascii\n')
                for point in points:
                    f.write(f'{point[0]} {point[1]} {point[2]} {point[3]}\n')

        elif 'camera' in topic:
            img_folder = os.path.join(output_parent_folder, topic.split('/')[1])
            os.makedirs(img_folder, exist_ok=True)
            np_arr = np.frombuffer(deserialized_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image_np is not None:
                cv2.imwrite(f"{img_folder}/{lidar_time:.6f}.jpg", image_np)

print(f"資料已儲存於 {output_parent_folder}")
