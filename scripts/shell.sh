#!/bin/bash
EXE=${@:-bash}
docker exec -it client /opt/ros/noetic/env.sh /opt/workspace/devel_isolated/env.sh /opt/ep_ws/devel/env.sh $EXE