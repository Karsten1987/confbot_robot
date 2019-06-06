# Copyright 2019 Mikael Arguedas
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ARG FROM_IMAGE=ubuntu:bionic
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update \
    && apt-get -qq upgrade -y \
    && apt-get install -q -y \
        bash-completion \
        dirmngr \
        gnupg2 \
        lsb-release \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-testing.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    colcon-common-extensions \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# FIXME This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

ARG ROS_DISTRO=dashing
ENV ROS_DISTRO=$ROS_DISTRO

# install ros_base
RUN apt-get -qq update && \
    apt-get -qq install ros-$ROS_DISTRO-ros-base -y && \
    rm -rf /var/lib/apt/lists/*

# setup underlay
ENV ROS2_UNDERLAY_WS /opt/ros2_underlay_ws
# # copy optional additional_repos.repos
# COPY ./additional_repos.repos $ROS2_UNDERLAY_WS/
RUN mkdir -p $ROS2_UNDERLAY_WS/src
WORKDIR $ROS2_UNDERLAY_WS

RUN git clone https://github.com/Karsten1987/roscon2018/ src/roscon2018
# RUN if [ -f additional_repos.repos ]; then vcs import src < additional_repos.repos; fi
# build underlay
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon \
    build \
    --merge-install \
    --cmake-args -DSECURITY=ON -DBUILD_TESTING=OFF --no-warn-unused-cli

ENV ROS_PACKAGE_PATH=$ROS2_UNDERLAY_WS/install/share:$ROS_PACKAGE_PATH


# install extra demo dependencies
RUN apt-get -qq update \
    && apt-get install -y \
        ros-$ROS_DISTRO-ros2bag \
        ros-$ROS_DISTRO-ros2component \
        ros-$ROS_DISTRO-rosbag2 \
        ros-$ROS_DISTRO-rosbag2-converter-default-plugins \
        ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
        ros-$ROS_DISTRO-rqt-graph \
        ros-$ROS_DISTRO-rqt-image-view \
        ros-$ROS_DISTRO-rqt-plot \
        ros-$ROS_DISTRO-rqt-publisher \
        ros-$ROS_DISTRO-rqt-service-caller \
        ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# setup ROS1 overlay

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list
ENV ROS2_OVERLAY_WS /opt/ros2_overlay_ws
RUN mkdir -p $ROS2_OVERLAY_WS/src/
WORKDIR $ROS2_OVERLAY_WS
RUN git clone --depth=1 https://github.com/ros2/rosbag2_bag_v2 $ROS2_OVERLAY_WS/src/rosbag2_bag_v2
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/melodic/setup.sh \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon \
       build \
       --merge-install \
       --cmake-args -DSECURITY=ON -DBUILD_TESTING=OFF --no-warn-unused-cli

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ['bash']
