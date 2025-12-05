import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_name = 'mobile_manipulator_description'
    pkg_gazebo = 'mobile_manipulator_gazebo'
    pkg_turtlebot_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # ------------------- 1. Launch Arguments 선언 -------------------
    # GUI 실행 여부를 인수로 받습니다. 기본값은 False로 설정하여 GUI를 끕니다.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',  # <--- 수정: 기본값을 'false'로 설정
        description='Set to "true" to run with Gazebo GUI.'
    )
    
    # 월드 파일 경로 인수 선언
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(get_package_share_directory(pkg_gazebo), 'world', 'empty.world'),
        description='Gazebo world file'
    )

    # LaunchConfiguration 객체 생성
    gui_config = LaunchConfiguration('gui')
    world_config = LaunchConfiguration('world_name')

    # ------------------- 2. URDF 파일 경로 설정 및 XML 변환 -------------------
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    # Rviz 시각화 및 Gazebo 스폰을 위해 URDF를 XML로 변환 (파일 생성)
    robot_description = ExecuteProcess(
        cmd=['xacro', xacro_file, '>', '/tmp/robot_description.urdf'],
        shell=True,
        output='screen'
    )
    
    # ------------------- 3. Gazebo 서버 실행 -------------------
    # turtlebot3_gazebo의 empty_world.launch.py를 호출하며 gui 인수를 전달합니다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_gazebo, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world': world_config,
            'gui': gui_config  # <--- 수정: gui 인수를 전달
        }.items()
    )

    # ------------------- 4. 로봇 모델을 Gazebo에 스폰(Spawn) -------------------
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-file', '/tmp/robot_description.urdf',
            '-entity', 'mobile_manipulator',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # ------------------- 5. Robot State Publisher -------------------
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': xacro_file}], # URDF 파일 경로를 직접 전달하거나,
    #     # parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
    #     output='screen'
    # )
    # 참고: 이전 로그에서 robot_state_publisher는 정상적으로 실행되었으므로 코드를 유지하거나 주석처리합니다.
    
    # ------------------- 6. Controller Manager 및 제어기 로드 -------------------
    # 제어기 로드 명령은 spawn_entity가 성공적으로 시작된 이후에 실행되도록 합니다.
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
        # spawn_entity 노드의 시작을 기다립니다.
        #on_action_start=[spawn_entity] 
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen',
        #on_action_start=[spawn_entity]
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen',
        #on_action_start=[spawn_entity]
    )

    return LaunchDescription([
        # 1. Arguments
        gui_arg,
        world_arg,
        
        # 2. URDF 변환
        robot_description,
        
        # 3. Gazebo 실행 (gzclient 비활성화)
        gazebo,
        
        # 4. 로봇 스폰
        spawn_entity,
        
        # 5. Controller 로드 (spawn 이후)
        load_joint_state_controller,
        load_diff_drive_controller,
        load_arm_controller
        
        # 참고: robot_state_publisher 노드가 누락된 경우 여기에 추가해야 합니다.
        # robot_state_publisher_node,
    ])
