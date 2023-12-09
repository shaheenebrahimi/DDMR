from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.actions.shutdown_action import ShutdownEvent 
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from ddmr.disc_robot import load_disc_robot
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

ROBOT = 'robot'
WORLD = 'world'

PACKAGE = 'ddmr'

def launch_setup(context):
    robot_file_name = LaunchConfiguration(ROBOT).perform(context)
    world_file_name = LaunchConfiguration(WORLD).perform(context)
    urdf = load_disc_robot(robot_file_name)['urdf']
   
    nodes = [
        Node(
            package=PACKAGE,
            executable='simulator_node',
            name='simulator_node',
            output='log',
            parameters=[{'robot': robot_file_name},
                        {'world': world_file_name}],
        ),
        Node(
            package=PACKAGE,
            executable='velocity_translator_node',
            name='velocity_translator_node',
            output='log',
        ),
        Node(
            package=PACKAGE,
            executable='navigation_controller_node',
            name='navigation_controller_node',
            output='log',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        )
    ]

    return nodes
def generate_launch_description():
    arg = DeclareLaunchArgument(ROBOT, default_value='robots/normal.robot')
    arg1 = DeclareLaunchArgument(WORLD, default_value='world/windy.world')

    return LaunchDescription([arg, arg1, OpaqueFunction(function = launch_setup)])

      
