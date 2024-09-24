import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource



from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_name = 'gazebo_pg'
    file_subpath = 'urdf/example_robot.urdf.xacro'


    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()



    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('gazebo_pg')) + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.path.join(os.path.join(get_package_share_directory('gazebo_pg')), 'materials', 'scripts') + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] 
    )

    node_joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')

    rviz_arg = ['-d' +  os.path.join(
            get_package_share_directory(pkg_name), 'rviz') + '/cfg.rviz']

      
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments = rviz_arg,
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
        rviz_node
    ])

