from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_simple",
            executable="publisher_test",
            name="publisher_test",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"update_frequency": 10.0}
            ]
        )
    ])