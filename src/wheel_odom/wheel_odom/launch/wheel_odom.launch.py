import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    wheel_odom_dir = get_package_share_directory('wheel_odom')
    
    # Create config file path
    config_file = os.path.join(wheel_odom_dir, 'params', 'wheel_odom_params.yaml')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        Node(
            package='wheel_odom',
            executable='encoder_publisher',
            name='encoder_publisher',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        Node(
            package='wheel_odom',
            executable='encoder_odometry',
            name='encoder_odometry',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])