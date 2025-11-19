import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_simulation = get_package_share_directory('box_simulation')
    
    # Paths
    world_file = os.path.join(pkg_box_simulation, 'worlds', 'underwater.world')
    model_path = os.path.join(pkg_box_simulation, 'models')
    
    # Set Gazebo model path
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )
    
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'box_panjang',
            '-file', os.path.join(model_path, 'box_panjang', 'model.sdf'),
            '-x', '0',
            '-y', '0',
            '-z', '1'
        ],
        output='screen'
    )
    
    # Controller node
    controller_node = Node(
        package='box_simulation',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )
    
    # Joy node (untuk joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_entity,
        controller_node,
        joy_node
    ])