# robotics-course

## build
from main directory
`colcon build`

## Run gazebo
run gazebo
`. install/setup.bash`
`ros2 launch bumperbot_description gazebo.launch.py`

## Run controller
Namespace is bumperbot_[0_9]
`. install/setup.bash`
`ros2 launch bumperbot_controller controller.launch.py namespace:=bumperbot_2`