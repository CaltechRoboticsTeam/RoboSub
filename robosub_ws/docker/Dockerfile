# ROS 2 Humble + Gazebo Harmonic — matches the Jetson's distro so sim code
# and vehicle code stay identical.
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# --- Gazebo Harmonic from the OSRF apt repo -------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && rm -rf /var/lib/apt/lists/*

# --- Sim + the ROS<->Gazebo bridge ----------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
 && rm -rf /var/lib/apt/lists/*

# --- Robot description, state estimation, teleop, debugging ---------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-robot-localization \
      ros-humble-tf2-tools \
      ros-humble-tf2-ros \
      ros-humble-teleop-twist-keyboard \
      ros-humble-joy \
      ros-humble-rqt-common-plugins \
      ros-humble-rqt-tf-tree \
      ros-humble-image-transport-plugins \
      ros-humble-vision-opencv \
      python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
      git vim tmux less iputils-ping net-tools \
 && rm -rf /var/lib/apt/lists/*

# --- Auto-source ROS and the workspace overlay in every shell -------------
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc \
 && echo 'export GZ_SIM_RESOURCE_PATH=/ws/src:${GZ_SIM_RESOURCE_PATH}' >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
