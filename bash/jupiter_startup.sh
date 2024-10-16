 #!/bin/bash

## SETUP ROS2 HUMBLE
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

## SETUP COLCON_CD
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

## SETUP PYENV - USED TO SELECT PYTHON VERSION 
export PYENV_ROOT="$HOME/.pyenv"
command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"

## SETUP PLATFORMIO
PATH="$PATH:$HOME/.platformio/penv/bin"

## SOURCE MICROROS WORKSPACE
source ~/uros_ws/install/setup.bash

## SETUP JUPITER ROBOT ROS2 WS
source ~/jupiter_ws/install/setup.bash

## SETUP LD LIDAR 
source ~/ldlidar_ros2_ws/install/setup.bash

# SETUP OPENAI GPT API KEY



ros2 launch jupiter_bringup bringup.launch.py &
