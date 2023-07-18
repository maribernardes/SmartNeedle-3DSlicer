from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    igtl_bridge = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"172.23.145.58"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    ld.add_action(igtl_bridge)
    return ld