import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'gazebo_pg'
    file_subpath = 'urdf/robot.urdf.xacro'

    # Process the robot description from xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Set environment variables for Gazebo
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('gazebo_pg')) + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('gazebo_pg'), 'materials', 'scripts') + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    # Node to publish robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]  # use_sim_time here
    )

    # Joint state publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]  # use_sim_time here
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen',
        parameters=[{'use_sim_time': True}]  # use_sim_time here
    )

    # Spawner nodes for controllers
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        parameters=[{'use_sim_time': True}]  # use_sim_time here
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        parameters=[{'use_sim_time': True}]  # use_sim_time here
    )

    # RViz node with configuration
    rviz_arg = ['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz') + '/cfg.rviz']
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_arg,
        output='screen'    )

    # Return the complete LaunchDescription
    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz_node
    ])
