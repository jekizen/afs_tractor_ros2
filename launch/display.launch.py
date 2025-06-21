import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('afs_robot')
    xacro_path = os.path.join(pkg_share, 'urdf', 'afs_robot.urdf.xacro')
    urdf_path = os.path.join(pkg_share, 'urdf', 'afs_robot.urdf')
    
    # Конвертируем Xacro в URDF
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_path, '-o', urdf_path],
        output='screen'
    )
    
    # Генерируем robot_description из URDF
    robot_description_content = Command([
        'cat', ' ', urdf_path
    ])
    
    return LaunchDescription([
        xacro_to_urdf,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'source_list': ['articulation_joint'],
                'rate': 30
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'config.rviz')],
        ),
    ])
