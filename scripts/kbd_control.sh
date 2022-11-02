#!/bin/bash
docker exec -it client /opt/ros/noetic/env.sh /opt/ep_ws/devel/env.sh roslaunch ep_teleop keyboard.launch
