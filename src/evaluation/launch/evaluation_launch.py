from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    evaluation = Node(
        package='evaluation',
        executable='evaluation_node',
        name='evaluation_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 8.0,
            'mass': 9.0877,
            'inertia_xx': 0.1454,
            'inertia_yy': 0.1366,
            'inertia_zz': 0.1594,
            'inertia_xy': 0.0,
            'inertia_xz': 0.0,
            'inertia_yz': 0.0,
            'center_of_mass': [0.0, 0.0, 0.0]
        }]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(evaluation)

    return launch_description
