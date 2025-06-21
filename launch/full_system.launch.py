import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Пути к файлам
    pkg_share = get_package_share_directory('afs_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'afs_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    controllers_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Получаем robot_description
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_path]
    )
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # 1. Основные ноды для визуализации
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # 2. Система управления
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[robot_description, controllers_config]
        ),
        
        # 3. Загрузка контроллеров (с задержкой)
        ExecuteProcess(
            cmd=['sleep', '2'],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller'],
            output='screen'
        ),
        
        # 4. Телеоперация с клавиатуры
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='gnome-terminal --',
            remappings=[
                ('/cmd_vel', '/velocity_controller/cmd_vel_unstamped')
            ]
        )
    ])
