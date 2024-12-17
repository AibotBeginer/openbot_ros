
Docker 运行 openbot ros

1. docker compose up -d --build
2. docker exec -it openbot_ros bash

```
# 环境
source /lzl_entrypoint.sh
renv
# 编译 openbot，失败则重试几次
colcon build --symlink-install --packages-up-to openbot
# 编译 openbot_ros    ，失败则重试几次
rbuild
# 缺依赖的话： sudo apt install -y ros-humble-xacro ros-humble-nav2-bringup ros-humble-grid-map ros-humble-grid-map-octomap ros-humble-perception-pcl os-humble-octomap-server
start_rosbridge &
rlaunch
# 查看 VNC： localhost:8080
# 查看 webViz (通过 rosbridge 转发)： https://webviz.io/app/?rosbridge-websocket-url=ws%3A%2F%2Flocalhost%3A9090
```