from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='video_publisher',
        executable='talker',
        name='video_publisher',
        output='screen',
    ),
    
        Node(
        package='video_subscriber',
        executable='listener',
        name='video_subscriber',
        output='screen',
    ),
    ])