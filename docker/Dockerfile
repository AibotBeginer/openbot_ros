FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
# https://github.com/ros-visualization/rviz/issues/1170#issuecomment-632188358
# https://github.com/orise-robotics/ros2-docker-devel/issues/36
# 

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


RUN apt-get update \
  && apt-get install -y ssh build-essential \
  gcc \
  g++ \
  git \
  gdb \
  clang \
  make \
  ninja-build \
  cmake \
  autoconf \
  automake \
  libtool \
  valgrind \
  locales-all \
  dos2unix \
  rsync \
  tar \
  && apt-get clean

RUN ( \
  echo 'LogLevel DEBUG2'; \
  echo 'PermitRootLogin yes'; \
  echo 'PasswordAuthentication yes'; \
  echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

RUN useradd -m user \
  && yes password | passwd user

RUN usermod -s /bin/bash user && echo 'source /opt/ros/humble/setup.bash' >> /home/user/.bashrc

# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && /usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_test_clion && exec bash"]

ENTRYPOINT ["/bin/bash"]
