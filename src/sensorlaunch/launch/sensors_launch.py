import os
import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensorlaunch'),
            'launch/drivers/xsens_mti_launch.py'))
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensorlaunch'),
            'launch/drivers/velodyne_launch.py'))
    )

    novatel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensorlaunch'),
            'launch/drivers/novatel_launch.py'))
    )

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensorlaunch'),
            'launch/drivers/usb_cam_launch.py'))
    )

    can_publisher_node = Node(
        package='sensorlaunch',
        executable='can_publisher_node',
        output='screen'
    )

    return launch.LaunchDescription([
        xsens_launch,
        velodyne_launch,
        novatel_launch,
        usb_cam_launch,
        can_publisher_node
    ])
