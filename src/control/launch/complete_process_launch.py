from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('control')

    return LaunchDescription([
        Node(
            package='control',
            executable='complete_process',
            name='complete_process',
            output='screen',
            emulate_tty=True,
            parameters=[
		        pkg_path + '/config/complete_process.yaml',
	        ],
        ),
    ])
