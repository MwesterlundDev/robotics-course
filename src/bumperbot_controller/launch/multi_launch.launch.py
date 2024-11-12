from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def controller_launcher(context, *args, **kwargs):
  count = int(LaunchConfiguration("count").perform(context))

  print(count)

  controllers = []
  for i in range(count):
    controller = Node(
      package="bumperbot_controller",
      executable="controller.launch.py",
      arguments=["namespace", f"bumperbot_{i}"]
    )
    controllers.append(controller)

  return controllers

def generate_launch_description():
  count_arg = DeclareLaunchArgument(
    "count",
    default_value="1"
  )

  controller_launch = OpaqueFunction(function=controller_launcher)

  return LaunchDescription([
    count_arg,
    controller_launch
  ])