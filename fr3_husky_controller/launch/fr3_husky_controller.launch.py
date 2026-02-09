import os
import yaml
import tempfile

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


def _normalize_robot_sides(robot_sides):
    if not robot_sides:
        return []
    normalized = [side.strip().lower() for side in robot_sides if side.strip()]
    if len(normalized) == 1 and normalized[0] == 'dual':
        return ['left', 'right']
    if 'dual' in normalized:
        raise RuntimeError("robot_side cannot mix 'dual' with other values.")
    return normalized
    
def _launch_setup(context, *args, **kwargs):
    # LaunchConfigurations (evaluate with context)
    robot_side_raw = LaunchConfiguration('robot_side').perform(context)
    robot_sides = _parse_robot_side(robot_side_raw)
    robot_sides = _normalize_robot_sides(robot_sides)
    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    namespace = LaunchConfiguration('namespace')

    # Ensure load_gripper is treated as a boolean (not a string) when applied to controller params.
    def _to_bool(value: str) -> bool:
        return value.lower() in ('true', '1', 'yes', 'y')
    load_gripper_value = _to_bool(load_gripper.perform(context))

    if not robot_sides:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    allowed = {"left", "right"}
    normalized_sides = robot_sides
    if any(side not in allowed for side in normalized_sides):
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    if len(set(normalized_sides)) != len(normalized_sides):
        raise RuntimeError("robot_side entries must be unique.")

    is_dual = len(normalized_sides) == 2
    if is_dual and set(normalized_sides) != allowed:
        raise RuntimeError("robot_side must contain both 'left' and 'right' for dual mode.")

    # URDF
    if is_dual:
        franka_xacro_file = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots', 'dual_fr3_husky.urdf.xacro'
        )
        robot_description_config = Command([
            FindExecutable(name='xacro'), ' ',
            franka_xacro_file,
            ' ros2_control:=true',
            ' with_sc:=false',
            ' fix_finger:=false',
            ' hand:=', load_gripper,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' virtual_joint:=false',
            ' as_two_wheels:=false',
        ])
    else:
        franka_xacro_file = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots', 'single_fr3_husky.urdf.xacro'
        )
        robot_description_config = Command([
            FindExecutable(name='xacro'), ' ',
            franka_xacro_file,
            ' ros2_control:=true',
            ' with_sc:=false',
            ' fix_finger:=false',
            ' hand:=', load_gripper,
            ' side:=', normalized_sides[0],
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' virtual_joint:=false',
            ' as_two_wheels:=false',
        ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}
    
    # RViz
    rviz_full_config = os.path.join(
        get_package_share_directory('fr3_husky_controller'),
        'rviz', 'fr3_husky.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
        ],
    )
    
    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    # ros2_control node (side-specific controller yaml file path)
    controllers_yaml = load_yaml('fr3_husky_controller', 'config/fr3_husky_ros_controllers.yaml')
    if controllers_yaml is None:
        raise RuntimeError("Failed to load fr3_husky_ros_controllers.yaml")
    controllers_root = controllers_yaml.get('/**', controllers_yaml)
    def _set_controller_params(ctrl_name):
        if ctrl_name not in controllers_root:
            raise RuntimeError(f"Controller '{ctrl_name}' not found in fr3_husky_ros_controllers.yaml")
        controllers_root[ctrl_name]['ros__parameters']['hand'] = load_gripper_value
    if is_dual:
        _set_controller_params('test_dual_fr3_husky_controller')
    else:
        controller_name = 'test_left_fr3_husky_controller' if normalized_sides[0] == 'left' else 'test_right_fr3_husky_controller'
        _set_controller_params(controller_name)

    # Ensure franka_robot_state_broadcaster is declared and uses the correct prefix
    controller_manager = controllers_root.get('controller_manager')
    if controller_manager is None or 'ros__parameters' not in controller_manager:
        raise RuntimeError("controller_manager parameters missing in fr3_husky_ros_controllers.yaml")
    cm_params = controller_manager['ros__parameters']
    cm_params.setdefault(
        'franka_robot_state_broadcaster',
        {'type': 'franka_robot_state_broadcaster/FrankaRobotStateBroadcaster'}
    )

    franka_state_params = controllers_root.setdefault(
        'franka_robot_state_broadcaster', {}).setdefault('ros__parameters', {})
    franka_state_params.setdefault('arm_id', 'fr3')
    # Prefix must match the ros2_control hardware 'name_stem' (prefix + arm_id)
    franka_state_params['interface_prefix'] = '' if is_dual else normalized_sides[0]

    # Write patched YAML to a temp file so ros2_control_node reads it with the correct schema.
    temp_fd, controllers_yaml_path = tempfile.mkstemp(prefix='fr3_husky_controllers_', suffix='.yaml')
    with os.fdopen(temp_fd, 'w') as f:
        yaml.safe_dump(controllers_yaml, f)

    if is_dual:
        joint_state_remap = 'dual_fr3_husky/joint_states'
    else:
        joint_state_remap = normalized_sides[0] + '_fr3_husky/joint_states'
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, controllers_yaml_path],
        remappings=[('joint_states', joint_state_remap)],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )
    
    # Load controllers
    load_controllers = []
    if is_dual:
        controllers = ['test_dual_fr3_husky_controller', 'joint_state_broadcaster']
    else:
        controller_name = 'test_left_fr3_husky_controller' if normalized_sides[0] == 'left' else 'test_right_fr3_husky_controller'
        controllers = [controller_name, 'joint_state_broadcaster']
    for controller in controllers:
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
            'source_list': (
                ['dual_fr3_husky/joint_states', 'left_franka_gripper/joint_states', 'right_franka_gripper/joint_states']
                if is_dual
                else [normalized_sides[0] + '_fr3_husky/joint_states', normalized_sides[0] + '_franka_gripper/joint_states']
            ),
            'rate': 30
        }],
    )

    franka_robot_state_broadcaster = None
    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
        )
    
    # gripper (only if you actually want it; and consider gating with load_gripper)
    gripper_launch_files = []
    if is_dual:
        for side in ['left', 'right']:
            robot_ip = '172.16.5.5' if side == 'left' else '172.16.6.6'
            gripper_launch_files.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
                    ),
                    launch_arguments={
                        'robot_ip': robot_ip,
                        'use_fake_hardware': use_fake_hardware,
                        'namespace': namespace,
                        'hand_prefix': side,
                    }.items(),
                )
            )
    else:
        robot_ip = '172.16.5.5' if normalized_sides[0] == 'left' else '172.16.6.6'
        gripper_launch_files.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
                ),
                launch_arguments={
                    'robot_ip': robot_ip,
                    'use_fake_hardware': use_fake_hardware,
                    'namespace': namespace,
                    'hand_prefix': normalized_sides[0],
                }.items(),
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

    # Send twist_mux output to the correct controller name (left/right/dual)
    if is_dual:
        cmd_vel_out_target = "/test_dual_fr3_husky_controller/cmd_vel_unstamped"
    else:
        cmd_vel_out_target = f"/test_{normalized_sides[0]}_fr3_husky_controller/cmd_vel_unstamped"

    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", cmd_vel_out_target)],
        parameters=[filepath_config_twist_mux],
    )

    # Launch husky_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_joy.launch.py'])))

    nodes = [
        rviz_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_publisher,
        launch_husky_control,
        node_twist_mux,
        launch_husky_teleop_joy,
        
    ]
    if franka_robot_state_broadcaster is not None:
        nodes.append(franka_robot_state_broadcaster)
    nodes.extend(gripper_launch_files)
    nodes.extend(load_controllers)
    return nodes
    
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_side',
            default_value="left",
            description="Robot side: left, right, or dual",
        ),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot'),
        DeclareLaunchArgument('load_gripper', default_value='true', description='Load gripper (true/false)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands'),
        DeclareLaunchArgument('db', default_value='False', description='Database flag'),
        OpaqueFunction(function=_launch_setup),
    ])
