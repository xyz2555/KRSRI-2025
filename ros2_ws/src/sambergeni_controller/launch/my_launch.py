from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sambergeni_controller',  # Ganti dengan nama paket Anda
            executable='node1',    # Nama sesuai dengan yang ada di setup.py
            name='node1'
        ),
        Node(
            package='sambergeni_controller',  # Ganti dengan nama paket Anda
            executable='node2',    # Nama sesuai dengan yang ada di setup.py
            name='node2'
        )
    ])
