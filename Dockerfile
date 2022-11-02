##########################################################################
FROM tb5zhh/icra-2023-client-base-carto:v2.0.0

ADD src /opt/ep_ws/src/rmus_solution
WORKDIR /opt/ep_ws
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
RUN apt-get update -q && \
     source /opt/workspace/devel_isolated/setup.bash && \
     ROS_OS_OVERRIDE=ubuntu:20.04:focal \
     DEBIAN_FRONTEND=noninteractive \        
     rosdep install -q --from-paths src --ignore-src --rosdistro noetic -y && \
     rm -rf /var/lib/apt/lists/* && \
     apt-get clean

RUN apt-get update && \
     apt-get install -y --no-install-recommends \
     ros-noetic-depthimage-to-laserscan ros-noetic-map-server python3-tf-conversions ros-noetic-global-planner && \
     rm -rf /var/lib/apt/lists/* && apt-get clean

RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple scipy

RUN source /opt/workspace/devel_isolated/setup.bash && catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF
ENV ENV_ROBOT_MODE=sim

ADD start.sh /opt/start.sh
CMD /opt/ros/noetic/env.sh /opt/ep_ws/devel/env.sh /opt/start.sh