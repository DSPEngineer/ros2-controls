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


################################################################################
# setup utc timeszone & install base ubuntu packages
RUN echo 'Etc/UTC' > /etc/timezone  \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get update && apt-get install -q -y --no-install-recommends \
         bash-completion \
         black \
         build-essential \
         curl \
         dirmngr \
         emacs \
         git \
         gnupg2 \
         iproute2 \
         net-tools \
         python-is-python3 \
         python3-pip \
         software-properties-common \
         ssh \
         sudo \
         tzdata \
         usbutils \
         wget \
         x11-apps \
         xauth \
  && add-apt-repository universe \
  && apt-get autoremove -y \
  && apt-get clean  \
  && rm -rf /var/lib/apt/lists/*


################################################################################
# setup ros package overlay & install ros packages
#####
# Gazebo Keys
#
RUN  curl -sSl https://packages.osrfoundation.org/gazebo.gpg \
          -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
           http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
#####
# ROS2 Keys
#
RUN  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
          -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
           http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       | tee /etc/apt/sources.list.d/ros2.list > /dev/null
#####
# ROS2 Packages
#
RUN  apt-get update \
  && apt-get install -q -y --no-install-recommends \
         python3-colcon-common-extensions \
         python3-colcon-mixin \
         python3-debugpy \
         python3-rosdep \
         python3-vcstool \
         ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
         ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
         ros-${ROS_DISTRO}-desktop \
         ros-${ROS_DISTRO}-joint-state-publisher \
         ros-${ROS_DISTRO}-joint-state-publisher-gui \
         ros-${ROS_DISTRO}-ros-gz \
         ros-${ROS_DISTRO}-ros2-control \
         ros-${ROS_DISTRO}-ros2-controllers \
         ros-${ROS_DISTRO}-xacro \
  && apt-get autoremove -y \
  && apt-get clean  \
  && rm -rf /var/lib/apt/lists/*


################################################################################
# setup colcon mixin and metadata
RUN  colcon mixin add default \
       https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default \
  && colcon metadata add default \
       https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
  && colcon metadata update


################################################################################
## Build and Install Dynamixel SDK 
WORKDIR /tmp/sdk
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
  && cd DynamixelSDK/c++/build/linux64 \
  && make \
  && make install

################################################################################
## Build and Install plotjuggler, from source

##
## Install required packages:
##
RUN  apt update \
  && apt install -y -q \
             binutils-dev \
             cmake \
             protobuf-compiler \
             liblua5.1-0-dev \
             libmosquittopp-dev \
             liblz4-dev \
             libprotoc-dev \
             libzmq3-dev \
             libqt5opengl5-dev \
             libqt5svg5-dev \
             libqt5websockets5-dev \
             libqt5x11extras5-dev \
             libzstd-dev \
             nlohmann-json3-dev \
             qtbase5-dev \
             ros-${ROS_DISTRO}-ament-cmake \
             ros-${ROS_DISTRO}-ament-cmake-core \
             ros-${ROS_DISTRO}-ament-cmake-export-dependencies \
             ros-${ROS_DISTRO}-ament-cmake-libraries \
             ros-${ROS_DISTRO}-ament-cmake-python  \
             ros-${ROS_DISTRO}-ament-cmake-ros \
             ros-${ROS_DISTRO}-ament-cmake-target-dependencies \
             ros-${ROS_DISTRO}-ament-package \
             ros-${ROS_DISTRO}-rclcpp \
             ros-${ROS_DISTRO}-rclcpp-components \
             ros-${ROS_DISTRO}-rosidl-default-generators \
             ros-${ROS_DISTRO}-rosidl-default-runtime \
  && apt-get autoremove -y \
  && apt-get clean  \
  && rm -rf /var/lib/apt/lists/*

  ##
  ## Build PlotJuggler, from source:
  ##     https://docs.ros.org/en/jazzy/p/plotjuggler_ros/
  ##
RUN  mkdir -p /opt/plotjuggler/src  \
  && cd /opt/plotjuggler/src  \
  && git clone --branch 0.2.3  https://github.com/PlotJuggler/plotjuggler_msgs.git  \
  && git clone --branch 3.10.9 https://github.com/facontidavide/PlotJuggler.git  \
  && git clone --branch 2.3.1  https://github.com/PlotJuggler/plotjuggler-ros-plugins.git  \
  && cd /opt/plotjuggler  \
  && . /opt/ros/${ROS_DISTRO}/setup.sh  \
  && export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}  \
  && export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
  && export ROS_VERSION=2  \
  && export ROS_PYTHON_VERSION=3  \
  && colcon build \
            --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DROS_DISTRO=${ROS_DISTRO} \
            -DROS_VERSION=2 \
            -DCMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}"


########################################################################
## Install VSCode: (or code-insiders) directly from Microsoft
RUN  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg \
  && install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg \
  && echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null \
  && rm -f packages.microsoft.gpg \
  && apt update \
  && apt install code -q -y \
  && apt-get autoremove -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/


########################################################################
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
    -G users,nogroup,dialout,sudo,adm \
    -m ${USERNAME} \
  && passwd -d ${USERNAME} \
  && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash"                                >> ${HOME_DIR}/.bashrc \
  && echo "source /opt/plotjuggler/install/setup.bash"                              >> ${HOME_DIR}/.bashrc \
  && echo "SETUP=\"\$(find ${WORKSPACE} -name setup.bash)\""                        >> ${HOME_DIR}/.bashrc \
  && echo "[ -n \"\${SETUP}\" ] && source \${SETUP}"                                >> ${HOME_DIR}/.bashrc \
  && echo "[[ \$LD_LIBRARY_PATH != */usr/local/lib* ]] && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib"  >> ${HOME_DIR}/.bashrc \
  && echo "[[ \$LD_LIBRARY_PATH != */opt/plotjuggler/install/plotjuggler/lib/plotjuggler* ]] && export LD_LIBRARY_PATH=/opt/plotjuggler/install/plotjuggler/lib/plotjuggler:\$LD_LIBRARY_PATH"  >> ${HOME_DIR}/.bashrc \
  && echo "[[ \$PATH != */opt/plotjuggler/install/plotjuggler/lib/plotjuggler* ]] && export PATH=/opt/plotjuggler/install/plotjuggler/lib/plotjuggler:\$PATH"   >> ${HOME_DIR}/.bashrc \
  && chown -R ${USERNAME}: ${HOME_DIR}

########################################################################
# create workspace and source dir
RUN mkdir -p ${WORKSPACE} \
  && chown  -R  ${USERNAME}:${USERNAME}  ${HOME_DIR}
WORKDIR ${WORKSPACE}

# setup dds config
ADD ./dds_config ${DDS_CONFIG_DIR}

# enable either cyclone dds or fast rtps

# copy code into workspace and set ownership to user
ADD --chown=${USERNAME}:${USERNAME} ./src ${WORKSPACE}/src

########################################################################
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
