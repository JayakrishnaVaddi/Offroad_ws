import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def random_location() -> float:
    y = random.uniform(-50, 50)
    return y

def generate_launch_description():

    # Gazebo launch file (from gazebo_ros package)
    gazebo_world = '/home/ubuntu/JK/ASU/MAE_598_AV/Offroad_ws/src/vehicle_bringup/worlds/forest_more.world'

    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    robot_description = '/home/ubuntu/JK/ASU/MAE_598_AV/Offroad_ws/src/vehicle_bringup/models/my_robot/model.sdf'

    spawn_y = random_location()

    return LaunchDescription([
        # Start Gazebo with the specified world
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', gazebo_launch_file],
        #     output='screen'),   

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': gazebo_world}.items(),
        ),

        # Spawn the TurtleBot with scaling
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-x', '-50', '-y', '-40', '-z', '0.1',
                '-database', 'turtlebot3_burger', # Assuming using the 'turtlebot3_burger' model
                # '-y', '20.0', '-z', '20.0', '-z', '20.0'  # Scale the robot size 1.5 times in all dimensions
            ],
            output='screen'
        )
    ])
