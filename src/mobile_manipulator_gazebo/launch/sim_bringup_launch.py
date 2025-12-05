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
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Set to "true" to run with Gazebo GUI.'
    )
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(get_package_share_directory(pkg_gazebo), 'world', 'empty.world'),
        description='Gazebo world file'
    )

    gui_config = LaunchConfiguration('gui')
    world_config = LaunchConfiguration('world_name')

    # ------------------- 2. URDF 파일 경로 설정 및 XML 변환 -------------------
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    robot_description = ExecuteProcess(
        cmd=['xacro', xacro_file, '>', '/tmp/robot_description.urdf'],
        shell=True,
        output='screen'
    )
    
    # ------------------- 3. Gazebo 서버 실행 -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_gazebo, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world': world_config,
            'gui': gui_config
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

    # ------------------- 5. Controller Manager 노드 실행 (추가) -------------------
    # ⚠️ 수정 1: Controller Manager를 실행하는 노드 추가
    # mobile_manipulator_gazebo/config/ 디렉토리에 컨트롤러 설정 파일이 있다고 가정합니다.
    controller_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'mobile_manipulator_controllers.yaml' # <--- 실제 YAML 파일명 확인 후 필요시 수정
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )
    # -------------------------------------------------------------------

    # ------------------- 6. Controller 로드 명령 -------------------
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
        # on_action_start 인수는 제거된 상태 유지
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen',
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen',
    )

    return LaunchDescription([
        # 1. Arguments
        gui_arg,
        world_arg,
        
        # 2. URDF 변환
        robot_description,
        
        # 3. Gazebo 실행 (gzserver만 실행)
        gazebo,
        
        # 4. 로봇 스폰
        spawn_entity,
        
        # 5. Controller Manager 노드 실행 <--- 이 순서가 중요합니다!
        control_node, 

        # 6. Controller 로드 (이제 control_node가 서비스를 제공합니다)
        load_joint_state_controller,
        load_diff_drive_controller,
        load_arm_controller
        
        # robot_state_publisher 노드는 gzserver를 포함하는 launch 파일에서 이미 실행되고 있을 가능성이 높습니다.
    ])