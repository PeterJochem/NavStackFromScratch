import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Controls if the simulation (Gazebo) clock is used.')
    use_jsp = DeclareLaunchArgument('use_jsp', default_value='true', description='Controls if the joint state publisher is launched to publish default joint values.')
    rviz_arg = DeclareLaunchArgument('start_rviz', default_value='true', description='Controls whether to start rviz2 or not.')
    rviz_config_dir = DeclareLaunchArgument('rviz_config_dir', default_value='true', description='Location of RVIZ config.')
    use_jsp = DeclareLaunchArgument('use_jsp', default_value='true', description='Controls whether the joint state publisher is used to publish default joint states.')
        

    # Configure the robot state publisher.
    urdf_file_name = "turtlebot3_burger.urdf"
    print(f"urdf_file_name : urdf_file_name")
    urdf = os.path.join(
        get_package_share_directory('turtle_description'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    rsp_params = {'robot_description': robot_description}
    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[rsp_params, {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}]) 

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', [os.path.join(get_package_share_directory('turtle_description'), 'config', 'config.rviz')]], on_exit=launch.actions.Shutdown())
    joint_state_publisher = Node(package="joint_state_publisher",
                                executable="joint_state_publisher",
                                name="joint_state_publisher")


    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value="false", description='Use simulation (Gazebo) clock if true'),
                                rviz_arg,
                                use_jsp,
                                rviz_node,
                                joint_state_publisher,
                                robot_state_publisher
                                ])

generate_launch_description()
