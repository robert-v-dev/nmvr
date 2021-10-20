from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    map_builder = Node(
        package="sim",
        executable="map_builder"
    )

    csv_read = Node(
        package="sim",
        executable="csv_read",
    )

    csv_write = Node(
        package="sim",
        executable="csv_write"
    )
    grid_visualiser = Node(
        package="sim",
        executable="grid_visualiser"
    )

    ld.add_action(map_builder)
    ld.add_action(csv_read)
    ld.add_action(csv_write)
    ld.add_action(grid_visualiser)

    return ld