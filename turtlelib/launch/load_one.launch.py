from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, \
                                 TextSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
          DeclareLaunchArgument(name='use_jsp', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if joint_state_publisher is launched'),
          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),
          DeclareLaunchArgument(name='color', default_value='light_black',
                                choices=['light_black', 'red', 'green', 'blue'],
                                description='Change the color of the turtlebot'),

          SetLaunchConfiguration(name='config_file',
                                 value=[TextSubstitution(text='basic_'),
                                        LaunchConfiguration('color'),
                                        TextSubstitution(text='.rviz')]),
          SetLaunchConfiguration(name='model',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('turtlelib'),
                                    'urdf',
                                    'turtlebot3_burger.urdf.xacro'])),
          SetLaunchConfiguration(name='rvizconfig',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('turtlelib'),
                                    'config',
                                    LaunchConfiguration('config_file')])),

          Node(package='joint_state_publisher',
               executable='joint_state_publisher',
               condition=LaunchConfigurationEquals('use_jsp', 'true'),
               namespace=PathJoinSubstitution([LaunchConfiguration('color')])),

          Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description':
                            ParameterValue(Command(['xacro ',
                                                    LaunchConfiguration('model'),
                                                    ' color:=',
                                                    LaunchConfiguration('color')]),
                                           value_type=str)}],
               namespace=PathJoinSubstitution([LaunchConfiguration('color')])),

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=['-d', LaunchConfiguration('rvizconfig'),
                          '-f', PathJoinSubstitution([LaunchConfiguration('color'), 'base_link'])],
               condition=LaunchConfigurationEquals('use_rviz', 'true'),
               namespace=PathJoinSubstitution([LaunchConfiguration('color')]),
               on_exit=Shutdown())])