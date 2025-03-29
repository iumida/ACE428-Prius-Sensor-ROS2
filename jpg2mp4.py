import cv2
import os

# 取得使用者輸入
input_folder = input("請輸入包含 JPG 圖像的資料夾路徑: ")
frame_rate = int(input("請輸入影片的幀數 (FPS): "))
output_video = input("請輸入輸出影片檔案名稱 (例如 output.mp4): ")

# 確認資料夾存在
if not os.path.exists(input_folder):
    print("資料夾不存在！")
    exit()

# 取得資料夾內的所有 JPG 圖片
image_files = [f for f in os.listdir(input_folder) if f.endswith('.jpg')]

# 確保圖片按檔名排序
image_files.sort()

# 確認資料夾內有圖片
if not image_files:
    print("資料夾內沒有 JPG 圖片！")
    exit()

# 讀取第一張圖片來確定影像大小
first_image = cv2.imread(os.path.join(input_folder, image_files[0]))
height, width, _ = first_image.shape

# 設定影片的編碼格式與輸出
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用 mp4 編碼
out = cv2.VideoWriter(output_video, fourcc, frame_rate, (width, height))

# 依序將每張圖片寫入影片
for image_file in image_files:
    img_path = os.path.join(input_folder, image_file)
    img = cv2.imread(img_path)
    
    if img is None:
        print(f"無法讀取圖片: {img_path}")
        continue

    out.write(img)

# 釋放影片資源
out.release()

print(f"影片已儲存至 {output_video}")
