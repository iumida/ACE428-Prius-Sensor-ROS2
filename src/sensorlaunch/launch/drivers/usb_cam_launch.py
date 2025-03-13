import launch
import os
import glob
from launch_ros.actions import Node

def generate_launch_description():
    base_config_path = '/home/prius428/sensorlaunch/src/sensorlaunch/config'

    # 可用的相機設備（僅使用 video0, video2, video4, video6, ...）
    camera_configs = [
        {
            'device': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0',
            'camera_name': 'f',
            'camera_info_url': f'file://{base_config_path}/camera_info_f.yaml'
        },
        {
            'device': '/dev/v4l/by-path/pci-0000:3e:00.0-usb-0:1.2:1.0-video-index0',
            'camera_name': 'bl',
            'camera_info_url': f'file://{base_config_path}/camera_info_bl.yaml'
        },
        {
            'device': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:7:1.0-video-index0',
            'camera_name': 'fl',
            'camera_info_url': f'file://{base_config_path}/camera_info_fl.yaml'
        },
        {
            'device': '/dev/v4l/by-path/pci-0000:3e:00.0-usb-0:2.1:1.0-video-index0',
            'camera_name': 'b',
            'camera_info_url': f'file://{base_config_path}/camera_info_b.yaml'
        },
        {
            'device': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:5:1.0-video-index0',
            'camera_name': 'br',
            'camera_info_url': f'file://{base_config_path}/camera_info_br.yaml'
        },
        {
            'device': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.0-video-index0',
            'camera_name': 'fr',
            'camera_info_url': f'file://{base_config_path}/camera_info_fr.yaml'
        }
    ]

    # 解析 /dev/v4l/by-path/... 為 /dev/videoX
    available_devices = glob.glob('/dev/video*')
    for cam in camera_configs:
        resolved_path = os.path.realpath(cam['device'])
        if resolved_path in available_devices:
            cam['device'] = resolved_path
        else:
            print(f"Skipping unavailable device: {resolved_path}")

    usb_cam_nodes = []
    for cam in camera_configs:
        namespace = f'camera_{cam["camera_name"]}'
        usb_cam_nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                namespace=namespace,
                name=f'usb_cam_node_{cam["camera_name"]}',
                parameters=[{
                    'video_device': cam['device'],
                    'framerate': 10.0,
                    'io_method': 'mmap',
                    'frame_id': namespace,
                    'pixel_format': 'uyvy2rgb',
                    'image_width': 1920,
                    'image_height': 1280,
                    'camera_name': cam['camera_name'],
                    'camera_info_url': cam['camera_info_url']
                }],
                remappings=[
                    ('image_raw', f'/{namespace}/image_raw'),
                    ('camera_info', f'/{namespace}/camera_info')
                ],
                output='screen'
            )
        )

    return launch.LaunchDescription(usb_cam_nodes)
