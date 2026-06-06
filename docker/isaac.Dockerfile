# Isaac Sim 4.5 + ROS 2 Humble — solves the "ROS2 Bridge startup failed" crash
# we hit on the base image (the bridge dlopens rclpy at extension-load time
# regardless of whether we enable it from Python).

FROM nvcr.io/nvidia/isaac-sim:4.5.0

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg2 lsb-release locales software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-${ROS_DISTRO}-ros-base \
       ros-${ROS_DISTRO}-rcl \
       ros-${ROS_DISTRO}-rclpy \
       ros-${ROS_DISTRO}-vision-msgs \
       ros-${ROS_DISTRO}-tf2-ros \
       ros-${ROS_DISTRO}-sensor-msgs \
       ros-${ROS_DISTRO}-geometry-msgs \
       python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Isaac Sim looks for ROS 2 libs on LD_LIBRARY_PATH — source on every shell start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:${PATH}

WORKDIR /isaac-sim
