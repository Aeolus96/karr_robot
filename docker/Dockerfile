# K.A.R.R. ROS 2 Jazzy dev image (Ubuntu 24.04). Multi-arch: builds natively on amd64 and arm64
# (x86 laptops, Apple Silicon, Jetson). No platform is pinned; Docker picks the host's.
# Official ros:jazzy-ros-base is multi-arch; osrf/ros:jazzy-desktop is amd64 only, so desktop is added via apt.
FROM ros:jazzy-ros-base

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

# tigervnc/novnc/fluxbox: browser GUI for hosts without a usable X server (macOS)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-urdf-tutorial \
      ros-jazzy-urdf-launch \
      ros-jazzy-urdfdom-py \
      liburdfdom-tools \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-numpy python3-matplotlib \
      mesa-utils \
      tigervnc-standalone-server novnc python3-websockify fluxbox xterm \
      sudo git nano less bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 images ship an 'ubuntu' user with uid/gid 1000. Remap it to the host user so files written
# into the bind-mounted repo keep the right owner on Linux hosts (dev.sh passes your uid/gid).
RUN if [ "${USER_GID}" != "1000" ]; then groupmod -o -g ${USER_GID} ${USERNAME}; fi \
    && if [ "${USER_UID}" != "1000" ]; then usermod -o -u ${USER_UID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME}/ros2_ws/src \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

COPY ws-env.sh /usr/local/bin/ws-env
COPY entrypoint.sh /usr/local/bin/karr-entrypoint
RUN sed -i 's/\r$//' /usr/local/bin/ws-env /usr/local/bin/karr-entrypoint \
    && chmod 0755 /usr/local/bin/ws-env /usr/local/bin/karr-entrypoint \
    && echo 'source /usr/local/bin/ws-env' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN rosdep update --rosdistro jazzy || true

ENTRYPOINT ["/usr/local/bin/karr-entrypoint"]
CMD ["sleep", "infinity"]
