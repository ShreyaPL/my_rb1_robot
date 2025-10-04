#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "my_rb1_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'my_rb1_robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['cat ', robot_desc_path])}]
    )

    # # Adding new launch arguement for model_name
    # declare_model_name = DeclareLaunchArgument("model_name", default_value="my_robot",
    #                                             description="Name of spawned robot model")
    # # Spawn the Robot #
    # declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
    #                                         description="Model Spawn X Axis Value")
    # declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
    #                                         description="Model Spawn Y Axis Value")
    # declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5",
    #                                         description="Model Spawn Z Axis Value")
    declare_rviz_config = DeclareLaunchArgument("rviz_config", default_value="",
                                                description="Absolute path to RViz2 config")

        # Joint State Publisher GUI #
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_args = []
    rviz_args += ['-d', rviz_config]

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below 
            # (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz2_node,
        ]
    )