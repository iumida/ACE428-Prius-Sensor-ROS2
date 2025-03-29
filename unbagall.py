import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import cv2
from tqdm import tqdm
import multiprocessing
import glob

##########################################
# 全局變數：儲存各 topic 輸出的 timestamp（字串格式，"%.6f"）
##########################################
processed_timestamps = {}

def init_processed_timestamps(topics):
    global processed_timestamps
    for topic in topics:
        processed_timestamps[topic] = set()

##########################################
# 基本設定
##########################################
bag_folder = input("請輸入 ROS 2 bag 資料夾路徑: ")
output_parent_folder = input("請輸入資料保存位置 (例如 /home/user/output): ")
os.makedirs(output_parent_folder, exist_ok=True)
core_count = int(input("請輸入要使用的核心數: "))

# 要處理的所有 topics
all_topics = [
    '/velodyne_points',
    '/camera_b/image_raw/compressed', '/camera_bl/image_raw/compressed',
    '/camera_br/image_raw/compressed', '/camera_f/image_raw/compressed',
    '/camera_fl/image_raw/compressed', '/camera_fr/image_raw/compressed',
    '/steering_can', '/velocity_can', '/novatel/oem7/bestpos',
    '/imu/data', '/filter/euler'
]
init_processed_timestamps(all_topics)

# 設定時間容忍範圍 (秒)
time_tolerance = 0.15

##########################################
# Step 1: 初步掃描 - 收集各 topic 的 timestamp
##########################################
print("進行初步掃描，收集各 topic 的 timestamp ...")
storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# 以浮點數收集，每個 topic 的所有 timestamp
prelim_ts = {topic: [] for topic in all_topics}
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic in prelim_ts:
        prelim_ts[topic].append(timestamp/1e9)
reader = None

# 選出訊息數量最少的 topic 作為基準
baseline_topic = min(prelim_ts, key=lambda k: len(prelim_ts[k]))
baseline_ts = sorted(prelim_ts[baseline_topic])
print(f"基準 topic: {baseline_topic}，訊息數量: {len(baseline_ts)}")

# 去除頭尾各2秒的資料，僅保留有效區間
if baseline_ts:
    t_start = baseline_ts[0] + 2.0
    t_end = baseline_ts[-1] - 2.0
    valid_baseline = [t for t in baseline_ts if t >= t_start and t <= t_end]
else:
    valid_baseline = []
valid_baseline_str = set(f"{t:.6f}" for t in valid_baseline)

# 將全局共同 timestamp 以基準 topic 為依據（這裡只採用基準 topic 的有效 timestamp作為共同時間軸）
global_common = valid_baseline_str.copy()
print(f"全局共同 timestamp 數量: {len(global_common)}")

##########################################
# Step 2: 取得 topics 型態資訊（供反序列化使用）
##########################################
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
topics_info = reader.get_all_topics_and_types()
topic_types = {topic.name: topic.type for topic in topics_info}
reader = None

##########################################
# Step 3: 對其他 topic 取得輸出起點
# 每個 topic 必須找到與全局共同 timestamp 最早值最接近的時間作為起點
##########################################
topic_start_time = {}
T0 = float(sorted(global_common, key=lambda x: float(x))[0]) if global_common else None
# 基準 topic 的起點即為全局共同最早值
if valid_baseline:
    topic_start_time[baseline_topic] = valid_baseline[0]
else:
    topic_start_time[baseline_topic] = T0

for topic in all_topics:
    if topic == baseline_topic:
        continue
    sorted_ts = sorted(prelim_ts[topic])
    if sorted_ts:
        # 找出該 topic 中與 T0 差距最小的 timestamp作為起點
        closest = min(sorted_ts, key=lambda t: abs(t - T0)) if T0 is not None else sorted_ts[0]
        topic_start_time[topic] = closest
    else:
        topic_start_time[topic] = T0
    print(f"Topic: {topic} 輸出起點: {topic_start_time[topic]:.6f}")

# 後續處理以 global_common 為依據，轉換成 float 列表
base_times = sorted([float(t) for t in global_common])
total_base = len(base_times)
print(f"後續處理的共同 timestamp 數量: {total_base}")

##########################################
# Step 4: 定義平行處理函式
##########################################
def process_chunk(args):
    topic, buffer, base_times_chunk, output_parent_folder = args
    processed = 0
    for b in base_times_chunk:
        b_str = f"{b:.6f}"
        candidates = [(abs(t - b), i) for i, (t, _) in enumerate(buffer) if abs(t - b) <= time_tolerance]
        if not candidates:
            processed += 1
            continue
        _, best_idx = min(candidates, key=lambda x: x[0])
        msg_time, msg_data = buffer[best_idx]
        deserialized_msg = deserialize_message(msg_data, get_message(topic_types[topic]))
        if topic == '/velodyne_points':
            pc_dtype = np.dtype({
                'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                'offsets': [0, 4, 8, 12, 16, 18],
                'itemsize': 22
            })
            points_struct = np.frombuffer(deserialized_msg.data, dtype=pc_dtype)
            points = np.vstack((points_struct['x'],
                                points_struct['y'],
                                points_struct['z'],
                                points_struct['intensity'])).T
            path = os.path.join(output_parent_folder, 'pcd_files')
            os.makedirs(path, exist_ok=True)
            num_points = points.shape[0]
            with open(f"{path}/{b_str}.pcd", 'w') as f:
                f.write('# .PCD v0.7\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\n')
                f.write('TYPE F F F F\n')
                f.write('COUNT 1 1 1 1\n')
                f.write(f'WIDTH {num_points}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {num_points}\n')
                f.write('DATA ascii\n')
                for point in points:
                    f.write(f'{point[0]} {point[1]} {point[2]} {point[3]}\n')
        elif topic == '/novatel/oem7/bestpos':
            for folder, value in zip(['Latitude', 'Longitude'], [deserialized_msg.lat, deserialized_msg.lon]):
                path = os.path.join(output_parent_folder, folder)
                os.makedirs(path, exist_ok=True)
                with open(f"{path}/{b_str}.txt", 'w') as f:
                    f.write(str(value))
        elif topic in ['/steering_can', '/velocity_can']:
            path = os.path.join(output_parent_folder, topic.strip('/'))
            os.makedirs(path, exist_ok=True)
            value = deserialized_msg.steering_tire_angle if topic == '/steering_can' else deserialized_msg.longitudinal_velocity
            with open(f"{path}/{b_str}.txt", 'w') as f:
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
                with open(f"{path}/{b_str}.txt", 'w') as f:
                    f.write(str(value))
        elif topic == '/filter/euler':
            euler_data = ['imu_roll', 'imu_pitch', 'imu_yaw']
            values = [deserialized_msg.vector.x, deserialized_msg.vector.y, deserialized_msg.vector.z]
            for folder, value in zip(euler_data, values):
                path = os.path.join(output_parent_folder, folder)
                os.makedirs(path, exist_ok=True)
                with open(f"{path}/{b_str}.txt", 'w') as f:
                    f.write(str(value))
        elif '/camera_' in topic:
            camera_name = topic.split('/')[1]
            img_folder = os.path.join(output_parent_folder, camera_name)
            os.makedirs(img_folder, exist_ok=True)
            np_arr = np.frombuffer(deserialized_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image_np is not None:
                cv2.imwrite(f"{img_folder}/{b_str}.jpg", image_np)
        processed += 1
    return processed

##########################################
# Step 4: 正式處理各 topic
# 設定批次讀取數量：
#   - /velodyne_points 一次讀取 300 筆；
#   - /camera_... 一次讀取 500 筆；
#   - 其他 TXT 主題一次讀取全部（以極大值模擬）
# 並只處理 timestamp 大於該 topic 輸出起點的資料
# 這裡我們加入重疊邏輯：每次處理完畢後，保留最後 overlap_time 秒內的資料以避免邊界漏讀
##########################################
def process_topic(topic):
    print(f"\n開始處理 topic: {topic}")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    if topic == '/velodyne_points':
        batch_size = 1000
    elif topic.startswith('/camera_'):
        batch_size = 700
    else:
        batch_size = int(1e9)
    
    start_time = topic_start_time[topic]
    chunk_buffer = []   # 本批次資料 (格式：(timestamp, data))
    base_index = 0      # 在 global base_times 中的進度（只處理 >= start_time 的資料）
    while base_index < len(base_times) and base_times[base_index] < start_time:
        base_index += 1
    
    pbar_progress = tqdm(total=total_base - base_index, desc=f"[{topic}] 已處理資料", position=1)
    pbar_chunk = tqdm(desc=f"[{topic}] 批次讀取", leave=False, position=2)
    
    # 定義重疊時間 (秒)
    overlap_time = 0.2
    
    while reader.has_next():
        t, data, timestamp_ns = reader.read_next()
        if t != topic:
            continue
        ts = timestamp_ns / 1e9
        if ts < start_time:
            continue
        chunk_buffer.append((ts, data))
        pbar_chunk.update(1)
        if len(chunk_buffer) >= batch_size:
            t_min = chunk_buffer[0][0]
            t_max = chunk_buffer[-1][0]
            current_base_times = []
            while base_index < len(base_times) and base_times[base_index] < t_min:
                base_index += 1
            temp_index = base_index
            while temp_index < len(base_times) and base_times[temp_index] <= t_max:
                current_base_times.append(base_times[temp_index])
                temp_index += 1
            # 僅保留在 global_common 中的 timestamp
            current_base_times = [t for t in current_base_times if f"{t:.6f}" in global_common]
            if current_base_times:
                processed_timestamps[topic].update({f"{t:.6f}" for t in current_base_times})
                num_processes = min(core_count, len(current_base_times))
                base_chunks = [current_base_times[i::num_processes] for i in range(num_processes)]
                args_list = [(topic, chunk_buffer, chunk, output_parent_folder) for chunk in base_chunks]
                pool = multiprocessing.Pool(processes=num_processes)
                for _ in tqdm(pool.imap(process_chunk, args_list),
                              total=len(args_list),
                              desc=f"[{topic}] 批次平行處理",
                              leave=False, position=3):
                    pass
                pool.close()
                pool.join()
                pbar_progress.update(len(current_base_times))
            # 重疊策略：保留最後 overlap_time 秒內的資料
            t_cut = t_max - overlap_time
            chunk_buffer = [msg for msg in chunk_buffer if msg[0] > t_cut]
    pbar_chunk.close()
    reader = None

    if chunk_buffer:
        t_min = chunk_buffer[0][0]
        t_max = chunk_buffer[-1][0]
        current_base_times = []
        while base_index < len(base_times) and base_times[base_index] < t_min:
            base_index += 1
        temp_index = base_index
        while temp_index < len(base_times) and base_times[temp_index] <= t_max:
            current_base_times.append(base_times[temp_index])
            temp_index += 1
        current_base_times = [t for t in current_base_times if f"{t:.6f}" in global_common]
        if current_base_times:
            processed_timestamps[topic].update({f"{t:.6f}" for t in current_base_times})
            num_processes = min(core_count, len(current_base_times))
            base_chunks = [current_base_times[i::num_processes] for i in range(num_processes)]
            args_list = [(topic, chunk_buffer, bc, output_parent_folder) for bc in base_chunks]
            pool = multiprocessing.Pool(processes=num_processes)
            for _ in tqdm(pool.imap(process_chunk, args_list),
                          total=len(args_list),
                          desc=f"[{topic}] 剩餘部分處理",
                          leave=False, position=3):
                pass
            pool.close()
            pool.join()
            pbar_progress.update(len(current_base_times))
    pbar_progress.close()
    print(f"完成處理 topic: {topic}")

##########################################
# Step 5: 後處理比對與確認刪除
##########################################
def post_process_compare_and_delete():
    print("----- 正式處理前比對結果 -----")
    for topic in all_topics:
        ts_set = processed_timestamps.get(topic, set())
        print(f"Topic: {topic} 輸出資料數量: {len(ts_set)}")
        extra = sorted(list(ts_set - global_common), key=lambda x: float(x))
        if extra:
            print(f"  多餘 timestamp (將移除): {extra}")
            ans = input(f"是否刪除 Topic {topic} 多出的 {len(extra)} 個檔案？(y/n): ")
            if ans.lower() == 'y':
                mapping = {
                    '/velodyne_points': [(os.path.join(output_parent_folder, 'pcd_files'), '.pcd')],
                    '/camera_b/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_b'), '.jpg')],
                    '/camera_bl/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_bl'), '.jpg')],
                    '/camera_br/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_br'), '.jpg')],
                    '/camera_f/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_f'), '.jpg')],
                    '/camera_fl/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_fl'), '.jpg')],
                    '/camera_fr/image_raw/compressed': [(os.path.join(output_parent_folder, 'camera_fr'), '.jpg')],
                    '/steering_can': [(os.path.join(output_parent_folder, 'steering_can'), '.txt')],
                    '/velocity_can': [(os.path.join(output_parent_folder, 'velocity_can'), '.txt')],
                    '/novatel/oem7/bestpos': [(os.path.join(output_parent_folder, 'Latitude'), '.txt'),
                                               (os.path.join(output_parent_folder, 'Longitude'), '.txt')],
                    '/imu/data': [(os.path.join(output_parent_folder, key), '.txt') for key in
                                  ['imu_roll_acc', 'imu_pitch_acc', 'imu_yaw_acc', 'imu_acc_x', 'imu_acc_y', 'imu_acc_z']],
                    '/filter/euler': [(os.path.join(output_parent_folder, key), '.txt') for key in
                                      ['imu_roll', 'imu_pitch', 'imu_yaw']]
                }
                if topic in mapping:
                    for folder, ext in mapping[topic]:
                        if os.path.exists(folder):
                            files = glob.glob(os.path.join(folder, f"*{ext}"))
                            for f in files:
                                base = os.path.basename(f)
                                name, _ = os.path.splitext(base)
                                if name in extra:
                                    os.remove(f)
                                    print(f"刪除檔案: {f}")
        else:
            print("  無多餘 timestamp。")

##########################################
# 主程式
##########################################
if __name__ == "__main__":
    print("\n開始初步比對，僅保留全局共同 timestamp 作為後續處理依據。")
    print(f"全局共同 timestamp 數量: {len(global_common)}")
    
    for topic in all_topics:
        process_topic(topic)
    
    print("\n✅ 全部 topic 處理完成！輸出資料位於:")
    print(output_parent_folder)
    print("開始後處理比對及確認刪除多餘檔案...")
    post_process_compare_and_delete()
    print("後處理完成！")
