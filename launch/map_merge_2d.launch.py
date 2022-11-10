import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("map_merge_2d")

    # Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='server',
        description='Host Name / Namespace')

    # Create Launch configurations
    namespace = LaunchConfiguration('namespace')
    
    # Topic remappings
    # remappings = [('/map', 'map'), 
    #                 ('/tf', 'tf')]

    # Declare launch actions
    start_map_merger = Node(
        package='map_merge_2d',
        executable='map_merger',
        name='map_merge',
        namespace=namespace,
        output='screen',
        # remappings=remappings,
        parameters=[
          ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
        # prefix=['xterm -e gdb -ex run --args'],
        emulate_tty=True)
 
    # Create Launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_arg_namespace)

    ld.add_action(start_map_merger)
    return ld