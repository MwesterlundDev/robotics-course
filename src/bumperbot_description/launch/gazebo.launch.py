import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def gen_robot_list(number_of_robots):
    robots = []

    for i in range(number_of_robots):
        robot_name = "bumperbot_%d" % i
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})


    return robots 


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
      is_ignition,
      "  robot_name:=", "bumperbot_0"
    ]), 
    value_type=str
  )

  robots = gen_robot_list(10)

  # robot_state_publisher = Node(
  #   package="robot_state_publisher",
  #   executable="robot_state_publisher",
  #   namespace="bumperbot_0",
  #   parameters=[{
  #      'frame_prefix': 'bumperbot_0/', 
  #      "robot_description": robot_description
  #      }]
  # )

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

  gz_spawn_entity_cmd = []

  for robot in robots:
    print(robot)

    robot_description = ParameterValue(
    Command([
      'xacro ', 
      LaunchConfiguration("model"),
      " is_ignition:=",
      is_ignition,
      "  namespace:=", robot['name'],
    ]), 
    value_type=str
  )

    robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      namespace=robot['name'],
      parameters=[{
          "robot_description": robot_description
        }]
    )

    gz_spawn_entity_cmd.append(robot_state_publisher)

    gz_spawn_entity_cmd.append(Node(
      package="ros_gz_sim",
      executable="create",
      # namespace=robot['name'],
      output="screen",
      arguments=[
                "-topic", f"/{robot['name']}/robot_description",
                "-name", robot['name'],
                "-x", str(robot['x_pose']),
                "-y", str(robot['y_pose']),
                "-z", str(robot['z_pose']),
                ]
    ))

  ld = LaunchDescription()

  ld.add_action(model_arg)
  # ld.add_action(robot_state_publisher)
  ld.add_action(gazebo_resource_path)
  ld.add_action(gazebo)

  for cmd in gz_spawn_entity_cmd:
    ld.add_action(cmd)

  return ld