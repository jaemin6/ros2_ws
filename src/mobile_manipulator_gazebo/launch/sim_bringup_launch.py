import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition



def generate_launch_description():

    pkg_desc = 'mobile_manipulator_description'
    pkg_gazebo = 'mobile_manipulator_gazebo'

    # GUI argument
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo with GUI'
    )

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(
            get_package_share_directory(pkg_gazebo),
            'world',
            'empty.world'
        ),
        description='Gazebo world file'
    )

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world_name')

    # ---------------------- XACRO 처리 ----------------------
    xacro_file = os.path.join(
        get_package_share_directory(pkg_desc),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            xacro_file
        ]),
        value_type=str
    )

    # ---------------------- gazebo server ----------------------
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--verbose'
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # ---------------------- robot_state_publisher ----------------------
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ---------------------- controller manager ----------------------
    controller_yml = os.path.join(
        get_package_share_directory(pkg_desc),
        'config',
        'controller_manager.yaml'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_yml
        ],
        output='screen'
    )

    # ---------------------- spawn robot ----------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator'
        ],
        output='screen'
    )

    # ---------------------- load controllers ----------------------
    load_joint_state = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'diff_cont'],
        output='screen'
    )

    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        world_arg,
        gazebo_server,
        gazebo_client,
        robot_state_pub,
        control_node,
        spawn_entity,
        load_joint_state,
        load_diff,
        load_arm
    ])
