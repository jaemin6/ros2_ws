import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'mobile_manipulator_description'
    pkg_gazebo = 'mobile_manipulator_gazebo'
    pkg_turtlebot_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # 1. URDF 파일 경로
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    # 2. Rviz에서 시각화를 위해 URDF를 파싱하여 XML로 변환
    robot_description = ExecuteProcess(
        cmd=['xacro', xacro_file, '>', '/tmp/robot_description.urdf'],
        shell=True
    )
    
    # 3. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name')}.items()
    )

    # 4. 로봇 모델을 Gazebo에 스폰(Spawn)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', '/tmp/robot_description.urdf',
                                   '-entity', 'mobile_manipulator',
                                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
                        output='screen')

    # 5. Controller Manager 및 제어기 로드
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 6. 바퀴 구동 제어기 로드 (Nav2용)
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen'
    )
    
    # 7. 매니퓰레이터 궤적 제어기 로드 (MoveIt!용)
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value=os.path.join(get_package_share_directory(pkg_gazebo), 'world', 'empty.world'),
            description='Gazebo world file'
        ),
        robot_description,
        gazebo,
        spawn_entity,
        load_joint_state_controller,
        load_diff_drive_controller,
        load_arm_controller
    ])
