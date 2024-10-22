import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
  bumperbot_description_dir = get_package_share_directory("bumperbot_description")
  ros_distro = os.environ["ROS_DISTRO"]
  is_ignition = "True" if ros_distro == "humble" else "False"

  model_arg = DeclareLaunchArgument(
    name="model",
    default_value=os.path.join(bumperbot_description_dir, "urdf", "bumperbot_description.urdf.xacro"),
    description="Absolute path to robot URDF file"
  )

  robot_description = ParameterValue(
    Command([
      'xacro ', 
      LaunchConfiguration("model"),
      " is_ignition:=",
      is_ignition
    ]), 
    value_type=str
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}]
  )

  gazebo_resource_path = SetEnvironmentVariable(
    name="GZ_SIM_RESOURCE_PATH",
    value=[
      str(Path(bumperbot_description_dir).parent.resolve())
    ]
  )

  # Launch gazebo with empty world
  gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
    os.path.join(
      get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
    ]),
    launch_arguments=[
      ("gz_args", [" -v 4", " -r", " empty.sdf"])
    ]
  )

  gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=["-topic", "robot_description",
               "-name", "bumperbot"]
  )

  return LaunchDescription([
    model_arg,
    robot_state_publisher,
    gazebo_resource_path,
    gazebo,
    gz_spawn_entity
  ])