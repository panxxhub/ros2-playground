'''
    Launches the Fibonacci server node.
'''
from sys import prefix

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    '''
	    Launches the Fibonacci server node.
    '''
    client = Node(
        package='action_example',
        executable='minimal_client',
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "INFO",
            "--log-level",
            "minimal_action_client:=DEBUG",
        ],
    )
    server = Node(
        package='action_example',
        executable='minimal_server',
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "INFO",
        ],
    )

    #    prefix=["gdbserver localhost:12034"])
    return LaunchDescription([server, client])
