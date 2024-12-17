FROM robot-registry.jd.local/openbot/openbot-linuxarm64:1.2.0
# https://github.com/ros-visualization/rviz/issues/1170#issuecomment-632188358
# https://github.com/orise-robotics/ros2-docker-devel/issues/36

ARG DEBIAN_FRONTEND=noninteractive

RUN set -eu \
  && ln -snf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime && echo "Asia/Shanghai" > /etc/timezone \
  && sed -i -E 's/(archive|security).ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
  language-pack-zh-hans \
  curl \
  tzdata \
  vim \
  less

ENV LANG='zh_CN.UTF-8' LANGUAGE='zh_CN:zh' LC_ALL=''

RUN apt install -y wget python3 python3-pip software-properties-common ca-certificates \
  && pip3 install pyyaml distro

RUN echo "chooses:" > fish_install.yaml \
  && echo "- {choose: 1, desc: '():ROS(ROS/ROS2,Jetson)'}" >> fish_install.yaml \
  && echo "- {choose: 1, desc: ''}" >> fish_install.yaml \
  && echo "- {choose: 2, desc: ''}" >> fish_install.yaml \
  && echo "- {choose: 1, desc: 'humble'}" >> fish_install.yaml \
  && echo "- {choose: 1, desc: '1humble(ROS2), 2'}" >> fish_install.yaml \
  && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
  && rm -rf fish_install.yaml \
  && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
  && cat /tmp/fishros_install.log | tail -n 30 \
  && apt-get clean && apt autoclean

WORKDIR /workspaces/lzl_ws

# Clone repositories
RUN git clone -b develop-nav2 https://github.com/AibotBeginer/openbot.git src/openbot
RUN git clone -b develop-nav2 https://github.com/AibotBeginer/openbot_ros.git src/openbot_ros

# RUN apt remove --no-auto-remove -y libpcl-dev
# COPY pcl-1.12.tar.gz /workspaces/lzl_ws/pcl-1.12.tar.gz
# RUN tar -xzvf pcl-1.12.tar.gz && cd pcl && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j4 && make install
RUN apt update && apt install -y ros-humble-desktop ros-humble-pcl-conversions ros-humble-vision-msgs ros-humble-octomap \
 ros-humble-octomap-msgs ros-humble-rosbridge-suite ros-humble-perception-pcl ros-humble-octomap-server  ros-humble-grid-map ros-humble-grid-map-octomap \
 ros-humble-xacro ros-humble-nav2-bringup 

# Add environment setup to bashrc
RUN echo "#!/bin/bash" > /lzl_entrypoint.sh \ 
    && echo "export OPENBOT_HOME=/workspaces/lzl_ws" >> /lzl_entrypoint.sh \
    && echo "ros2env() { source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42 && echo \"ROS_DOMAIN_ID set to \$ROS_DOMAIN_ID\"; }" >> /lzl_entrypoint.sh \
    && echo "renv() { ros2env && source /opt/cyber/setup.bash && echo \"CyberRT Activated\"; }" >> /lzl_entrypoint.sh \
    && echo "c() { clear; }" >> /lzl_entrypoint.sh \
    && echo "rbuild() { renv && cd \$OPENBOT_HOME && export CMAKE_PREFIX_PATH=/usr/local/share/pcl-1.12:\$CMAKE_PREFIX_PATH && colcon build --symlink-install --packages-ignore openbot_simulator --packages-up-to openbot openbot_ros openbot_fake_robot; }" >> /lzl_entrypoint.sh \
    && echo "rlaunch() { renv && cd \$OPENBOT_HOME && source install/setup.sh && export LD_LIBRARY_PATH=/opt/ros/humble/lib:\$LD_LIBRARY_PATH && ros2 launch openbot_ros demo_nav2_octomap_openbot.launch.py; }" >> /lzl_entrypoint.sh \
    && echo "rpull() { cd \$OPENBOT_HOME/src/openbot && git pull && cd \$OPENBOT_HOME/src/openbot_ros && git pull; }" >> /lzl_entrypoint.sh \
    && echo "start_docs() { cd \$OPENBOT_HOME/docs/html && python3 -m http.server 8000 & xdg-open http://localhost:8000; }" >> /lzl_entrypoint.sh \
    && echo "start_rosbridge() { ros2 launch rosbridge_server rosbridge_websocket_launch.xml \\" >> /lzl_entrypoint.sh \
    && echo "  address:='0.0.0.0' \\" >> /lzl_entrypoint.sh \
    && echo "  port:='9090' \\" >> /lzl_entrypoint.sh \
    && echo "  topics_glob:='"[/goal, /goal_pose, /odom, /cmd_vel, /map*, /robot_description, /map_updates, /global_costmap/costmap*, /downsampled_costmap*, /plan, /global_costmap/published_footprint, /local_costmap/costmap*, /local_plan, /local_costmap/published_footprint, /marker, /waypoints, /tf, /rosout]"' \\" >> /lzl_entrypoint.sh \
    && echo "  retry_startup_delay:='5.0' \\" >> /lzl_entrypoint.sh \
    && echo "  fragment_timeout:='600' \\" >> /lzl_entrypoint.sh \
    && echo "  delay_between_messages:='0' \\" >> /lzl_entrypoint.sh \
    && echo "  max_message_size:='10000000' \\" >> /lzl_entrypoint.sh \
    && echo "  unregister_timeout:='10.0' \\" >> /lzl_entrypoint.sh \
    && echo "  use_compression:='false' \\" >> /lzl_entrypoint.sh \
    && echo "  bson_only_mode:='false'; }" >> /lzl_entrypoint.sh \
    && chmod +x /lzl_entrypoint.sh

CMD ["bash"]

## docker exec -it openbot_ros bash
# # 环境
# source /lzl_entrypoint.sh
# renv
# # 编译 openbot，失败则重试几次
# colcon build --symlink-install --packages-up-to openbot
# # 编译 openbot_ros    ，失败则重试几次
# rbuild
# # 缺依赖的话： sudo apt install -y ros-humble-xacro ros-humble-nav2-bringup ros-humble-grid-map ros-humble-grid-map-octomap ros-humble-perception-pcl ros-humble-octomap-server
# start_rosbridge &
# rlaunch
# 查看 VNC： localhost:8080
# 查看 webViz (通过 rosbridge 转发)： https://webviz.io/app/?rosbridge-websocket-url=ws%3A%2F%2Flocalhost%3A9090