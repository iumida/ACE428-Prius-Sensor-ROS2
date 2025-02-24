import launch
from launch_ros.actions import Node
import os

def generate_launch_description():
    base_video_devices = [0, 2, 4, 6, 8, 10]
    usb_cam_nodes = []

    def find_available_device(start):
        device_num = start
        while device_num < 50:
            if os.path.exists(f'/dev/video{device_num}'):
                return device_num
            device_num += 2
        return None

    for i, device_num in enumerate(base_video_devices):
        actual_device = find_available_device(device_num)
        if actual_device is None:
            print(f"No available device found starting from /dev/video{device_num}")
            continue

        usb_cam_nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                namespace=f'camera{i+1}',
                name=f'usb_cam_node_{i+1}',
                parameters=[{
                    'video_device': f'/dev/video{actual_device}',
                    'framerate': 10.0,
                    'io_method': 'mmap',
                    'frame_id': f'camera{i+1}',
                    'pixel_format': 'uyvy2rgb',
                    'image_width': 1920,
                    'image_height': 1280,
                    'camera_name': f'cam{i+1}',
                }],
                remappings=[
                    ('image_raw', f'/camera{i+1}/image_raw'),
                    ('camera_info', f'/camera{i+1}/camera_info')
                ],
                output='screen'
            )
        )

    return launch.LaunchDescription(usb_cam_nodes)
