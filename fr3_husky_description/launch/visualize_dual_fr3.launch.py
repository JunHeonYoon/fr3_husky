import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def robot_state_publisher_spawner(context: LaunchContext, load_gripper, load_mobile):
    load_gripper_str = context.perform_substitution(load_gripper)
    load_mobile_str = context.perform_substitution(load_mobile)
    franka_xacro_filepath = os.path.join(
        get_package_share_directory('fr3_husky_description'),
        'robots',
        'dual_fr3.urdf.xacro',
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath, mappings={'hand': load_gripper_str, 'mobile': load_mobile_str}
    ).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
    ]


def generate_launch_description():
    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    
    load_mobile_parameter_name = 'load_mobile'
    load_mobile = LaunchConfiguration(load_mobile_parameter_name)

    rviz_file = os.path.join(
        get_package_share_directory('fr3_husky_description'),
        'rviz',
        'visualize_dual_fr3.rviz',
    )

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[load_gripper, load_mobile]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='true',
                description='Use end-effector if true. Default value is franka hand. '
                'Robot is loaded without end-effector otherwise',
            ),
            DeclareLaunchArgument(
                load_mobile_parameter_name,
                default_value='false',
                description='Use Mobile Husky as base if true'
                'Base is loaded box-shaped otherwise',
            ),
            robot_state_publisher_spawner_opaque_function,
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', rviz_file],
            ),
        ]
    )
