FROM ros:humble-perception

ARG USERNAME=pupillabs
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_WS=/ws

# Create the user
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -g video -m $USERNAME
# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# below add the ubuntu packages you need inside the container
RUN apt-get update && apt-get install -y \
    build-essential \
    binutils \
    ca-certificates \
    cmake \
    cmake-qt-gui \
    curl \
    dbus-x11 \
    ffmpeg \
    gdb \
    gcc \
    g++ \
    gfortran \
    git \
    tar \
    lsb-release \
    procps \
    manpages-dev \
    unzip \
    zip \
    wget \
    xauth \
    swig \
    openexr \
    pkg-config \
    qv4l2 \
    v4l-utils \
    zlib1g-dev \
    locales \
    lua5.3 \
    liblua5.3 \
    libjchart2d-java \
    default-jdk \
    ninja-build \
    stow \
    libboost-all-dev \
    libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
    libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
    libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
    libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
    liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
    libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
    libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
    libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
    libvtk9-dev libdc1394-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev libopenexr-dev libgflags-dev libsuitesparse-dev\
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    python3-natsort \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-urg-node \
    ros-${ROS_DISTRO}-rviz2 && \
    locale-gen en_US.UTF-8 && \
    LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# handle python dependecies for your project in the related requirements file
COPY requirements/${ROS_DISTRO}.txt /tmp/requirements.txt
RUN python3 -m pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt

# fix wierd ROS docker memory issue
RUN echo "ulimit -n 524388" >> /home/${USERNAME}/.bashrc
RUN echo "ulimit -Hn 524388" >> /home/${USERNAME}/.bashrc
RUN echo "ulimit -n 524388" >> /root/.bashrc
RUN echo "ulimit -Hn 524388" >> /root/.bashrc

# add ros setup file to bashrc for convinience
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> /home/${USERNAME}/.bashrc
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> /root/.bashrc
# add new ros setup to bashrc if it exists
RUN echo "[ -f ${ROS_WS}/install/setup.bash ] && source ${ROS_WS}/install/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "[ -f ${ROS_WS}/install/setup.bash ] && source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

CMD ["/bin/bash"]
SHELL ["/bin/bash", "-c"]

# set up entrypoint
COPY entrypoints/${ROS_DISTRO}.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set permissions for user
RUN mkdir -p $ROS_WS && mkdir -p $ROS_WS/src
RUN chown -R ${USERNAME} ${ROS_WS}

# save username in env
ENV USER_NAME=${USERNAME}
ENV ROS_WS=${ROS_WS}

USER $USER_NAME