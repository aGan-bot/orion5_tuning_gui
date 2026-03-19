from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory("mujoco_pendulum"),
        "launch",
        "orion5_gravity_comp.launch.py",
    )

    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch))

    gui = Node(
        package="orion5_tuning_gui",
        executable="orion5_tuning_gui",
        output="screen",
    )

    return LaunchDescription([
        sim,
        gui,
    ])
