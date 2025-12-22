from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    linear_motor_controller_node = Node(
            package='linear_motor_controller',
            name='linear_motor_controller_node',
            executable='linear_motor_controller_node',
            parameters=[PathJoinSubstitution([FindPackageShare('linear_motor_controller'), 'config', 'linear_motor_params.yaml'])],
            output='screen',
    )

    return LaunchDescription([
        linear_motor_controller_node,
    ])