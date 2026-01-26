import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def _parse_robot_side(raw_value):
    if raw_value is None:
        return []
    raw_value = raw_value.strip()
    if not raw_value:
        return []
    try:
        parsed = yaml.safe_load(raw_value)
    except yaml.YAMLError:
        parsed = raw_value
    if isinstance(parsed, list):
        values = parsed
    elif isinstance(parsed, str):
        if ',' in parsed:
            values = [item.strip() for item in parsed.split(',') if item.strip()]
        else:
            values = [parsed.strip()] if parsed.strip() else []
    else:
        values = [str(parsed).strip()]
    return [value for value in values if value]


def _build_moveit_nodes(
    namespace,
    robot_description,
    robot_description_semantic,
    kinematics_yaml,
    moveit_simple_controllers_yaml,
    ompl_planning_yaml,
):
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_full_config = os.path.join(
        get_package_share_directory('fr3_husky_moveit_config'),
        'rviz', 'moveit.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    return run_move_group_node, rviz_node


def _launch_setup(context, *args, **kwargs):
    robot_side_raw = LaunchConfiguration('robot_side').perform(context)
    robot_sides = _parse_robot_side(robot_side_raw)

    if not robot_sides:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")

    normalized_sides = [side.strip().lower() for side in robot_sides if side.strip()]
    if not normalized_sides:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")

    if len(normalized_sides) == 1 and normalized_sides[0] == 'dual':
        normalized_sides = ['left', 'right']
    elif 'dual' in normalized_sides:
        raise RuntimeError("robot_side cannot mix 'dual' with other values.")

    allowed = {"left", "right"}
    if any(side not in allowed for side in normalized_sides):
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    if len(set(normalized_sides)) != len(normalized_sides):
        raise RuntimeError("robot_side entries must be unique.")

    if len(normalized_sides) == 2 and set(normalized_sides) != allowed:
        raise RuntimeError("robot_side must contain both 'left' and 'right' for dual mode.")
    if len(normalized_sides) not in (1, 2):
        raise RuntimeError("robot_side must have 1 entry (single) or 2 entries (dual).")

    namespace = LaunchConfiguration('namespace')
    load_gripper = LaunchConfiguration('load_gripper')
    load_mobile = LaunchConfiguration('load_mobile')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    ompl_planning_yaml = load_yaml(
        'fr3_husky_moveit_config',
        os.path.join('config', 'ompl_planning.yaml')
    ) or {}

    if len(normalized_sides) == 1:
        robot_side = normalized_sides[0]
        robot_ip = '172.16.5.5' if robot_side == 'left' else '172.16.6.6'

        franka_xacro_file = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots', 'single_fr3.urdf.xacro'
        )
        robot_description_config = Command([
            FindExecutable(name='xacro'), ' ',
            franka_xacro_file,
            ' ros2_control:=true',
            ' hand:=', load_gripper,
            ' side:=', robot_side,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' mobile:=', load_mobile,
        ])
        robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

        franka_semantic_xacro_file = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots', 'single_fr3.srdf.xacro'
        )
        robot_description_semantic_config = Command([
            FindExecutable(name='xacro'), ' ',
            franka_semantic_xacro_file,
            ' side:=', robot_side,
            ' hand:=', load_gripper,
            ' mobile:=', load_mobile,
        ])
        robot_description_semantic = {
            'robot_description_semantic': ParameterValue(robot_description_semantic_config, value_type=str)
        }

        kinematics_yaml = load_yaml(
            'fr3_husky_moveit_config',
            os.path.join('config', robot_side, 'kinematics.yaml')
        ) or {}

        moveit_simple_controllers_yaml = load_yaml(
            'fr3_husky_moveit_config',
            os.path.join('config', robot_side, 'single_fr3_controllers.yaml')
        ) or {}

        run_move_group_node, rviz_node = _build_moveit_nodes(
            namespace,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_simple_controllers_yaml,
            ompl_planning_yaml,
        )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='both',
            parameters=[robot_description],
        )

        ros2_controllers_path = os.path.join(
            get_package_share_directory('fr3_husky_moveit_config'),
            'config', robot_side, 'single_fr3_ros_controllers.yaml'
        )
        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[robot_description, ros2_controllers_path],
            remappings=[('joint_states', robot_side + '_fr3/joint_states')],
            output={'stdout': 'screen', 'stderr': 'screen'},
            on_exit=Shutdown(),
        )

        load_controllers = []
        for controller in ['single_fr3_arm_controller', 'joint_state_broadcaster']:
            load_controllers.append(
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'controller_manager', 'spawner', controller,
                        '--controller-manager-timeout', '60',
                        '--controller-manager',
                        PathJoinSubstitution([namespace, 'controller_manager'])
                    ],
                    output='screen'
                )
            )

        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'source_list': [
                    robot_side + '_fr3/joint_states',
                    robot_side + '_franka_gripper/joint_states'
                ],
                'rate': 30
            }],
        )

        franka_robot_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        )

        gripper_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
            ),
            launch_arguments={
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware,
                'namespace': namespace,
                'hand_prefix': robot_side,
            }.items(),
        )

        return [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_publisher,
            franka_robot_state_broadcaster,
            gripper_launch_file,
            *load_controllers,
        ]

    franka_xacro_file = os.path.join(
        get_package_share_directory('fr3_husky_description'),
        'robots', 'dual_fr3.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ',
        franka_xacro_file,
        ' ros2_control:=true',
        ' hand:=', load_gripper,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands,
        ' mobile:=', load_mobile,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('fr3_husky_description'),
        'robots', 'dual_fr3.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ',
        franka_semantic_xacro_file,
        ' hand:=', load_gripper,
        ' mobile:=', load_mobile,
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_config, value_type=str)
    }

    kinematics_yaml = load_yaml(
        'fr3_husky_moveit_config',
        os.path.join('config', 'dual', 'kinematics.yaml')
    ) or {}

    moveit_simple_controllers_yaml = load_yaml(
        'fr3_husky_moveit_config',
        os.path.join('config', 'dual', 'dual_fr3_controllers.yaml')
    ) or {}

    run_move_group_node, rviz_node = _build_moveit_nodes(
        namespace,
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        moveit_simple_controllers_yaml,
        ompl_planning_yaml,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('fr3_husky_moveit_config'),
        'config', 'dual', 'dual_fr3_ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'dual_fr3/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    load_controllers = []
    for controller in ['dual_fr3_arm_controller', 'dual_joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', "/controller_manager",
                ],
                output='screen'
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{
            'source_list': [
                'dual_fr3/joint_states',
                'left_franka_gripper/joint_states',
                'right_franka_gripper/joint_states'
            ],
            'rate': 30
        }],
    )

    franka_state_broadcasters = []
    for bc in ['left_franka_robot_state_broadcaster', 'right_franka_robot_state_broadcaster']:
        franka_state_broadcasters.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', bc,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', "/controller_manager",
                ],
                output='screen',
                condition=UnlessCondition(use_fake_hardware),
            )
        )

    left_gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
        ),
        launch_arguments={
            'robot_ip': '172.16.5.5',
            'use_fake_hardware': use_fake_hardware,
            'namespace': '',
            'hand_prefix': 'left'
        }.items(),
    )

    right_gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
        ),
        launch_arguments={
            'robot_ip': '172.16.6.6',
            'use_fake_hardware': use_fake_hardware,
            'namespace': '',
            'hand_prefix': 'right'
        }.items(),
    )

    return [
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
        *franka_state_broadcasters,
        left_gripper_launch_file,
        right_gripper_launch_file,
        *load_controllers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_side',
            default_value='left',
            description='Robot side: left, right, or dual',
        ),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot'),
        DeclareLaunchArgument('load_gripper', default_value='true', description='Load gripper (true/false)'),
        DeclareLaunchArgument('load_mobile', default_value='false', description='Load mobile base (true/false)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands'),
        DeclareLaunchArgument('db', default_value='False', description='Database flag'),
        OpaqueFunction(function=_launch_setup),
    ])
