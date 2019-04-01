FROM osrf/ros2:nightly

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src/roscon2018
COPY ./ci_script.bash $ROS2_WS
COPY ./workspace.repos $ROS2_WS
WORKDIR $ROS2_WS
RUN vcs import src < workspace.repos

CMD ["bash"]
