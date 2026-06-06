FROM osrf/ros:humble-desktop-full

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git vim nano sudo curl wget \
      python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-numpy python3-scipy python3-matplotlib python3-yaml \
      can-utils iproute2 \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros2-control \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-tf-transformations \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-urdfdom-py \
      python3-pykdl \
      mesa-utils libgl1-mesa-dri x11-apps \
    && pip install --no-cache-dir pybullet \
    && rm -rf /var/lib/apt/lists/*

RUN EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1 || true) \
 && if [ -n "$EXISTING_USER" ] && [ "$EXISTING_USER" != "${USERNAME}" ]; then \
      userdel -r "$EXISTING_USER" 2>/dev/null || true; \
    fi \
 && EXISTING_GROUP=$(getent group ${USER_GID} | cut -d: -f1 || true) \
 && if [ -n "$EXISTING_GROUP" ] && [ "$EXISTING_GROUP" != "${USERNAME}" ]; then \
      groupdel "$EXISTING_GROUP" 2>/dev/null || true; \
    fi \
 && groupadd -g ${USER_GID} ${USERNAME} \
 && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "[ -f /home/${USERNAME}/ros2_ws/install/setup.bash ] && source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
