# openbot_ros

## 简介

> **Robot Framework** without ros/ros2, use cyberRT node replace ros/ros2 node.



## 工程代码

```bash
mkdir -p openbot/src
cd openbot/src

# openbot
git clone https://github.com/AibotBeginer/openbot.git

# openbot_ros
git clone https://github.com/AibotBeginer/openbot_ros.git

```



## 安装依赖

* FastRTPS

```bash
git clone https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS
mkdir -p build && cd build && cmake ..
sudo make install

```

* TinyXML2

```bash
sudo apt install libtinyxml2-dev
```



##  编译

```bash
cd openbot
colcon build --symlink-install  --packages-up-to openbot_ros
```



## 运行





