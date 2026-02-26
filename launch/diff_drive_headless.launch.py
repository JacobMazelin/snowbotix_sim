import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Include the gz_sim launch with headless (server-only) mode
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -s diff_drive.sdf',
            'gz_sim_server': 'true',
            'gz_sim_gui': 'false'
        }.items()
    )
    
    return LaunchDescription([
        gz_sim_launch
    ])
