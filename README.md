# ROS2 with Foundation Models

## LLM features/Task Adaptation
- Chain-of-Thought Prompting
- Few-shot fine tuning
- In-context learning(ICL)
- re-prompting teqniques
- Fine tuning with LoRA
- Function call task adaptation

## LLM Capabalities
- Task Planning with LLM
- Motion Planning with LLM
- LLM as translators: AutoTAMP
- LMM as trajectory generators: https://arxiv.org/pdf/2310.11604 

## ROS2 LLM 
- This repository is derived from [ROS-LLM](https://github.com/Auromix/ROS-LLM). 
- Runs locally on machine without AWS

## Commands
- cd ros2_ws
- colcon build --symlink-install   
- ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py
- ros2 topic pub /llm_input_message std_msgs/msg/String "data: 'Prompt message goes here'" -1