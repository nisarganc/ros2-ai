# ROS2 packages with AI Models 

## Project Set-up
- git clone https://github.com/nisarganc/ros2-ai.git
- cd ros2-ai
- colcon build --symlink-install

## Turtles Random Walk   
- ros2 launch turtles_bringup move_turtle_robot.launch.py

## Turtles RGB input
- ros2 launch turtles_bringup perception_turtle_robot.launch.py

## Turtles with LLM
- ros2 launch turtles_bringup planning_withgpt.launch.py
- ros2 topic pub /llm_input_message std_msgs/msg/String "data: 'Prompt message goes here'" -1

## Acknowledgements
- Many thanks to the repository [ROS-LLM](https://github.com/Auromix/ROS-LLM).