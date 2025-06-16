FROM docker.io/library/ubuntu:noble

ARG USERNAME="seymour"
ARG HOME_DIR="/home/${USERNAME}"
ARG WORKSPACE="${HOME_DIR}/workspace"
ARG DDS_CONFIG_DIR="/opt/dds/config"
ARG DEBIAN_FRONTEND="noninteractive"
ARG RUN_AS_UID=1000
ARG RUN_AS_GID=1000

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO="jazzy"
ENV CYCLONEDDS_URI="${DDS_CONFIG_DIR}/cyclonedds.xml"
ENV FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_CONFIG_DIR}/fastrtps.xml"

# rmw implementation can be overridden at runtime
# RMW_IMPLEMENTATION -> "rmw_cyclonedds_cpp" | "rmw_fastrtps_cpp"
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# setup utc timeszone & install base ubuntu packages
RUN echo 'Etc/UTC' > /etc/timezone  \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    build-essential \
    curl \
    dirmngr \
    git \
    gnupg2 \
    python-is-python3 \
    python3-pip \
    software-properties-common \
    sudo \
    tzdata \
    x11-apps \
  && add-apt-repository universe \
  && rm -rf /var/lib/apt/lists/*

# setup ros package overlay & install ros packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rqt* \
  && rm -rf /var/lib/apt/lists/*

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update default && \
  colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update

# Remove default ubuntu user
# create non-root user with given username
# and allow sudo without password
# and setup default users .bashrc
RUN userdel -r ubuntu \
  && groupadd --gid $RUN_AS_GID ${USERNAME} \
  && useradd -rm \
    -d ${HOME_DIR} \
    -s /bin/bash \
    --gid ${RUN_AS_GID} \
    --uid ${RUN_AS_UID} \
    -m ${USERNAME} \
  && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source install/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc \
  && chown -R ${USERNAME}: ${HOME_DIR}

# create workspace and source dir
RUN mkdir -p ${WORKSPACE} \
  && chown -R ${USERNAME}: ${HOME_DIR}
WORKDIR ${WORKSPACE}

WORKDIR /tmp/sdk
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
  && cd DynamixelSDK/c++/build/linux64 \
  && make \
  && make install

# setup dds config
ADD ./dds_config ${DDS_CONFIG_DIR}

# enable either cyclone dds or fast rtps

# copy code into workspace and set ownership to user
ADD --chown=${USERNAME}:${USERNAME} ./src ${WORKSPACE}/src

# install deps as non-root user
WORKDIR ${WORKSPACE}
USER ${USERNAME}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && sudo apt-get update \
  && sudo rosdep init \
  && rosdep update --rosdistro ${ROS_DISTRO} \
  && rosdep install -y -r -i --from-paths ${WORKSPACE}/src \
  && sudo rm -rf /var/lib/apt/lists/*"

# by default hold container open in background
CMD ["tail", "-f", "/dev/null"]
