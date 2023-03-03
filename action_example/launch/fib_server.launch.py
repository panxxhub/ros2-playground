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
    fib_server_node = Node(package='action_example',
                           executable='fib_main',
                           output="both",
                           arguments=[
                               "--ros-args",
                               "--log-level",
                               "INFO",
                               "--log-level",
                               "minimal_action_client:=DEBUG",
                           ],)
                        #    prefix=["gdbserver localhost:12034"])
    return LaunchDescription([fib_server_node])
