from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("fr3_husky_controller"),
        "config",
        "husky_ros_controllers.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_husky_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    # Load controllers
    load_controllers = []
    controllers = ['test_husky_controller', 'joint_state_broadcaster']
    for controller in controllers:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Mirror husky_control/teleop_base.launch.py here so cmd_vel goes to test_husky_controller.
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "twist_mux.yaml"]
    )

    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", "/test_husky_controller/cmd_vel_unstamped")],
        parameters=[filepath_config_twist_mux],
    )

    # Launch husky_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_joy.launch.py'])))


    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    for controller in load_controllers:
        ld.add_action(controller)
    ld.add_action(launch_husky_control)
    ld.add_action(node_twist_mux)
    ld.add_action(launch_husky_teleop_joy)

    return ld
