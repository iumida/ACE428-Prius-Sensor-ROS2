import os
import glob

# -----------------------------
# 設定
# -----------------------------
folder_mapping = {
    "pcd_files": ".pcd",
    "camera_b": ".jpg",
    "camera_bl": ".jpg",
    "camera_br": ".jpg",
    "camera_f": ".jpg",
    "camera_fl": ".jpg",
    "camera_fr": ".jpg",
    "steering_can": ".txt",
    "velocity_can": ".txt",
    "Latitude": ".txt",
    "Longitude": ".txt",
    "imu_roll_acc": ".txt",
    "imu_pitch_acc": ".txt",
    "imu_yaw_acc": ".txt",
    "imu_acc_x": ".txt",
    "imu_acc_y": ".txt",
    "imu_acc_z": ".txt",
    "imu_roll": ".txt",
    "imu_pitch": ".txt",
    "imu_yaw": ".txt"
}

# -----------------------------
# 輸入
# -----------------------------
root = input("請輸入解出的資料資料夾路徑 (例如 /home/user/output): ")

# -----------------------------
# 讀取所有 timestamp
# -----------------------------
folder_timestamps = {}

print("\n========== 讀取資料夾 ==========")
for folder, ext in folder_mapping.items():
    full_path = os.path.join(root, folder)
    files = sorted(glob.glob(os.path.join(full_path, f"*{ext}")))
    timestamps = [os.path.splitext(os.path.basename(f))[0] for f in files]
    folder_timestamps[folder] = sorted([float(t) for t in timestamps])
    print(f"讀取 {folder} 完成，共 {len(timestamps)} 筆")

# -----------------------------
# 尋找最小數量
# -----------------------------
min_count = min(len(ts) for ts in folder_timestamps.values())
print(f"\n========== 將所有資料夾同步到 {min_count} 筆 ==========")

# -----------------------------
# 準備刪除列表
# -----------------------------
delete_list = {}

for folder, timestamps in folder_timestamps.items():
    if len(timestamps) > min_count:
        # 多餘的 timestamp
        excess = timestamps[min_count:]
        delete_list[folder] = excess

# -----------------------------
# 列出刪除計畫
# -----------------------------
if delete_list:
    print("\n========== 刪除列表 ==========")
    for folder, excess in delete_list.items():
        print(f"資料夾: {folder} 需刪除 {len(excess)} 筆，多餘 timestamps:")
        for t in excess:
            print(f"    {t:.6f} (與最後保留資料相差 {t - folder_timestamps[folder][min_count - 1]:.6f} 秒)")
else:
    print("\n✅ 無需刪除，所有資料夾已同步。")
    exit(0)

# -----------------------------
# 確認刪除
# -----------------------------
ans = input("\n是否確定刪除？(y/n): ")
if ans.lower() != 'y':
    print("❌ 已取消刪除")
    exit(0)

# -----------------------------
# 執行刪除
# -----------------------------
for folder, excess in delete_list.items():
    full_path = os.path.join(root, folder)
    ext = folder_mapping[folder]
    for t in excess:
        filepath = os.path.join(full_path, f"{t:.6f}{ext}")
        if os.path.exists(filepath):
            os.remove(filepath)
            print(f"刪除 {filepath}")

# -----------------------------
# 刪除後再次檢查
# -----------------------------
print("\n========== 刪除後資料夾狀態 ==========")
for folder, ext in folder_mapping.items():
    full_path = os.path.join(root, folder)
    files = sorted(glob.glob(os.path.join(full_path, f"*{ext}")))
    print(f"{folder} 剩餘 {len(files)} 筆")

print("\n✅ 刪除及同步完成！")
