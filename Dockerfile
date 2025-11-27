FROM ros:humble

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

WORKDIR /root/ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# src をコピー
COPY ./src ./src

# ビルド
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install

# 環境セットアップ
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
RUN ls -R /root/ros2_ws/install


SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "lifecycle_test", "moveit_test1"]