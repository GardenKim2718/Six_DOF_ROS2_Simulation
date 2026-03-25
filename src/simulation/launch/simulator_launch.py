from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    simulator = Node(
        package='simulation',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 8.0,
            'id': "Ego",
            'frame_id': "map",
            'initial_time': 0.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_z': 0.0,
            'initial_vx': 0.0,
            'initial_vy': 0.0,
            'initial_vz': 0.0,
            'initial_qx': 0.0,
            'initial_qy': 0.0,
            'initial_qz': 0.0,
            'initial_qw': 1.0,
            'initial_wx': 0.0,
            'initial_wy': 0.0,
            'initial_wz': 0.0,
            'initial_rwa_momentum': [0.0, 0.0, 0.0, 0.0],
            'mass': 9.0877,
            'inertia_xx': 0.1454,
            'inertia_yy': 0.1366,
            'inertia_zz': 0.1594,
            'inertia_xy': 0.0,
            'inertia_xz': 0.0,
            'inertia_yz': 0.0,
            'center_of_mass': [0.0, 0.0, 0.0],
            'max_thrust': 1.0,
            'max_rwa_momentum': 0.5,
            'max_rwa_torque': 0.055
        }]
    )
    
    display = Node(
        package='simulation',
        executable='display_node',
        name='display_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 8.0
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('simulation'),
                'resources',
                'six_dof_simulation.rviz'
            ])
        ]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(simulator)
    launch_description.add_action(display)
    launch_description.add_action(rviz)

    return launch_description