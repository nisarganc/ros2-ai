#!/bin/bash
#
# This script configures dependencies for ROS-LLM
# Version: 1.0
# Author: Herman Ye @Auromix
# Date: 2023-06-24

# Exit the script immediately if a command exits with a non-zero status
# set -x
set -e
# Install necessary dependencies for OpenAI
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3
sudo apt install -y python3-pip
pip install setuptools==58.2.0
pip install setuptools-rust==1.0.0
pip install openai==1.23.6
pip install -U openai-whisper
pip install catkin-pkg
sudo apt install gnome-terminal -y
pip install pysocks
pip install requests
sudo apt-get install libcanberra-gtk-module libcanberra-gtk3-module -y


pip install numpy = 1.21.2
pip install sounddevice
pip install pydub
pip install scipy
sudo apt install portaudio19-dev -y
sudo apt install ffmpeg -y

# Install dependencies for sounddevice/soundfile
sudo apt install libportaudio2 -y
sudo apt install alsa-utils -y
sudo apt install mpv -y
pip install numpy sounddevice cffi soundfile

# Check again
sudo apt update
sudo apt upgrade -y

# Turtlebot4 dependencies
sudo apt install ros-humble-irobot-create-msgs