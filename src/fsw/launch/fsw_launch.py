from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    navigation = Node(
        package='fsw',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'loop_rate_hz': 8.0}]
    )

    guidance = Node(
        package='fsw',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 8.0,
            'target_x': 10.0,
            'target_y': 5.0,
            'target_z': 0.0,
            'target_vx': 0.0,
            'target_vy': 0.0,
            'target_vz': 0.0,
            'target_qx': 0.0,
            'target_qy': 0.0,
            'target_qz': 0.25882,
            'target_qw': 0.96593,
            'target_wx': 0.0,
            'target_wy': 0.0,
            'target_wz': 0.0,
            'mass': 9.0877,
            'inertia_xx': 0.1454,
            'inertia_yy': 0.1366,
            'inertia_zz': 0.1594,
            'inertia_xy': 0.0,
            'inertia_xz': 0.0,
            'inertia_yz': 0.0,
            'max_force': 1.0,
            'max_torque': 1.0,
            'angular_kp': 0.08,
            'angular_kd': 0.4,
        }]
    )

    control = Node(
        package='fsw',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 8.0,
            'linear_kp': 0.16,
            'linear_kd': 0.8,
            'linear_ki': 0.0,
            'angular_kp': 0.08,
            'angular_kd': 0.4,
            'angular_ki': 0.0,
            'mass': 9.0877,
            'inertia_xx': 0.1454,
            'inertia_yy': 0.1366,
            'inertia_zz': 0.1594,
            'inertia_xy': 0.0,
            'inertia_xz': 0.0,
            'inertia_yz': 0.0,
            'max_force': 1.0,
            'max_torque': 1.0
        }]
    )

    actuator = Node(
        package='fsw',
        executable='actuator_node',
        name='actuator_node',
        output='screen',
        parameters=[{'loop_rate_hz': 8.0,
                     'max_thrust': 1.0,
                     'max_rwa_momentum': 0.5,
                     'max_rwa_torque': 0.055,
                     'lambda_qp': 0.01
        }]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(navigation)
    launch_description.add_action(TimerAction(period=0.005, actions=[guidance]))
    launch_description.add_action(TimerAction(period=0.010, actions=[control]))
    launch_description.add_action(TimerAction(period=0.020, actions=[actuator]))

    return launch_description