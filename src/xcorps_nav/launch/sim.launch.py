# X-CORPS
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    namespace_OS = '/OS'
    namespace_TS = '/TS'

    dt = 0.1
    Left_Bottom = [37.455842, 126.951699]
    Right_Bottom = [37.455934, 126.951688]
    Left_Top = [37.455834, 126.951573]
    Right_Top = [37.455925, 126.951562]
    origin = Left_Bottom

    ld = LaunchDescription()

    differentiater_node = Node(
        package='xcorps_nav',
        node_executable='differentiater.py',
        node_name='differentiater',
        output='screen'
    )

    gnss_converter_node = Node(
        package='xcorps_nav',
        node_executable='gnss_converter.py',
        node_name='gnss_converter',
        output='screen'
    )

    ld.add_action(differentiater_node)
    ld.add_action(gnss_converter_node)

    return ld
