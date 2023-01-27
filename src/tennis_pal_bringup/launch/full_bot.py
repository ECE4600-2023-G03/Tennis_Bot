

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    path_plan_node = Node(
        package="cpp_path_plan",
        executable="path_planner",
    )

    control_node = Node(
        package="cpp_path_plan",
        executable="control_loop",
    )

    detection = Node(
        package="camera_distributor",
        executable="distributor"
    )

    uart_bridge = Node(
        package="uart-bridge",
        executable="uart_bridge"
    )
    pilot = Node(
        package="pilot",
        executable="pilot"
    )

    state_machine = Node(
        package="state_machine",
        executable="state_machine"
    )

    ld.add_action(path_plan_node)
    ld.add_action(control_node)
    ld.add_action(detection)
    ld.add_action(uart_bridge)
    ld.add_action(pilot)
    ld.add_action(state_machine)

    return ld