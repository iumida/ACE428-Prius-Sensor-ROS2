import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    novatel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('novatel_oem7_driver'),
            'launch',
            'oem7_port.launch.py')),
        launch_arguments={
            'oem7_port_name': '/dev/ttyUSB1',
            'oem7_port_baud': '115200',
            'oem7_strict_receiver_init': 'False'
        }.items())
    return LaunchDescription([novatel_launch])
