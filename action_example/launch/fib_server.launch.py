'''
    Launches the Fibonacci server node.
'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    '''
	    Launches the Fibonacci server node.
    '''
    fib_server_node = Node(
        package='action_example',
        executable='fib_server',
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            "INFO",
            "--log-level",
            "minimal_action_client:=DEBUG",
        ],
    )
    return LaunchDescription([fib_server_node])
