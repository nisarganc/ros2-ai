# ROS2 packages with AI Models 

## Project Requirements

- **ROS2 Humble**

- **GLEW and GLFW Dependencies:**

  Install necessary packages, clone and build GLFW:

  ```bash
  sudo apt-get install libglew-dev
  sudo apt-get install libglfw3-dev
  sudo apt install libxinerama-dev libxcursor-dev xinput libxi-dev
  git clone https://github.com/glfw/glfw
  cd glfw
  cmake -G "Unix Makefiles"
  make
  sudo make install
  cd ..
  ```

- **GTSAM Repository:**

  Clone, checkout specific commit, build, install, and update `LD_LIBRARY_PATH`:

  ```bash
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  git checkout 618ac28f2cc407e27e9eaf4e36ece64bc236b8e7
  mkdir build && cd build
  cmake -DGTSAM_ALLOW_DEPRECATED_SINCE_V42=OFF .. # Disable deprecated functionality for compatibility
  sudo make install
  echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=/usr/local/share:${LD_LIBRARY_PATH}' >> ~/.bashrc
  source ~/.bashrc
  ```

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
- [MR.CAP](https://github.com/h2jaafar/mr.cap) for multi-robot planning and control algorithm.