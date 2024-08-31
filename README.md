# ROS2 packages with AI Models

## LLM 
- This repository is derived from [ROS-LLM](https://github.com/Auromix/ROS-LLM). 
- Runs locally on machine without AWS

## Commands
- cd ros2_ws
- colcon build --symlink-install   
- ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py
- ros2 topic pub /llm_input_message std_msgs/msg/String "data: 'Prompt message goes here'" -1