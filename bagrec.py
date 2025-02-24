import subprocess
import os

def get_next_bag_number():
    existing_files = [f for f in os.listdir('.') if f.startswith('ros2_bag_') and f.endswith('.db3')]
    numbers = sorted([int(f.split('_')[2].split('.')[0]) for f in existing_files if f.split('_')[2].split('.')[0].isdigit()])
    return numbers[-1] + 1 if numbers else 1

def record_ros2_bag():
    bag_number = get_next_bag_number()
    bag_filename = f"ros2_bag_{bag_number}.db3"
    
    topics = [
        "/camera1/image_raw/compressed",
        "/camera2/image_raw/compressed",
        "/camera3/image_raw/compressed",
        "/camera4/image_raw/compressed",
        "/camera5/image_raw/compressed",
        "/camera6/image_raw/compressed",
        "/imu/data",
        "/filter/euler",
        "/imu/acceleration",
        "/imu/angular_velocity",
        "/velodyne_points",
        "/steering_can",
        "/velocity_can",
        "/novatel/oem7/bestpos"
    ]
    
    command = ["ros2", "bag", "record"] + topics + ["-o", bag_filename]
    subprocess.run(command)

if __name__ == "__main__":
    record_ros2_bag()
