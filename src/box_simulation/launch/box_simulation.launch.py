import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg = get_package_share_directory("box_simulation")
    world = os.path.join(pkg, "worlds", "underwater.world")
    model_path = os.path.join(pkg, "models")

    os.environ["GAZEBO_MODEL_PATH"] = model_path

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gzserver", "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so", world],
            output="screen"
        ),

        ExecuteProcess(
            cmd=["gzclient"],
            output="screen"
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "box_panjang",
                "-file", os.path.join(model_path, "box_panjang", "model.sdf"),
                "-x", "0", "-y", "0", "-z", "1"
            ],
            output="screen"
        ),

        Node(
            package="joy",
            executable="joy_node",
            parameters=[{
                "dev": "/dev/input/js0",
                "deadzone": 0.05,
                "autorepeat_rate": 20.0
            }]
        ),
    ])
