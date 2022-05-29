# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:20.04
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash
#FROM $base_image
RUN echo base image: ${base_image}

#######################################################################
##                            Speeding up                            ##
#######################################################################
RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list

#######################################################################
##                      install common packages                      ##
#######################################################################
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
   pkg-config \
   apt-utils \
   wget \
   git \
   build-essential \ 
   net-tools \
   gedit \
   terminator \
   nautilus \
   software-properties-common \
   apt-transport-https \
   libopencv-dev \
   ffmpeg \
   x264 \
   libx264-dev \
   zip \
   unzip \
   usbutils \
   sudo \
   python3-pip \
   libusb-1.0-0-dev \
   dbus-x11

#######################################################################
##                           install font                            ##
#######################################################################
RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections 
RUN apt-get update && apt-get install -y ttf-mscorefonts-installer \
    ttf-ubuntu-font-family \
    msttcorefonts -qq

RUN python3 -m pip install --upgrade pip
RUN apt-get install  -y python3-ruamel.yaml

#######################################################################
##                       install nvidia docker                       ##
#######################################################################
RUN apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev \
    mesa-utils \
    x11-apps

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64


#######################################################################
##                            ros install                            ##
#######################################################################

# install packages
RUN apt-get update
RUN apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    curl

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt-get update 
RUN apt-get install -y \
    ros-noetic-desktop-full
 # install ros packages
ENV ROS_DISTRO noetic
COPY ./ros_entrypoint.sh /

RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \ 
    python3-rosinstall-generator \ 
    python3-wstool \
    build-essential 
#######################################################################
##                   install additional packages                     ##
#######################################################################
WORKDIR  /
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update 
RUN apt-get install -y libopencv-dev \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-teleop-twist-joy \
    ros-noetic-camera-calibration \
    ros-noetic-gmapping \
    ros-noetic-ros-numpy \
    ros-noetic-robot-state-publisher

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#######################################################################
##                       install azure kinect                        ##
#######################################################################

# Configuring the repositories
RUN curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN apt-add-repository https://packages.microsoft.com/ubuntu/20.04/prod
RUN apt-get update && apt-get install -y \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib-arm-linux-gnueabihf \
    g++-multilib-arm-linux-gnueabihf && \
   rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    freeglut3-dev \
    libgl1-mesa-dev \
    mesa-common-dev \
    libsoundio-dev \
    libvulkan-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxi-dev \
    libxrandr-dev \
    uuid-dev \
    libsdl2-dev \
    usbutils \
    libusb-1.0-0-dev \
    openssl \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ubuntu-x-swat/updates
RUN apt-get update && \
    apt-get dist-upgrade -y

RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN apt-get update
RUN echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections
RUN echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN apt-get install -y \
    libk4a1.4 \
    libk4a1.4-dev
RUN apt-get install -y \
    libk4abt1.1 \
    libk4abt1.1-dev

# RUN mkdir -p /etc/udev/rules.d/
# RUN cp /home/Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/99-k4a.rules
# RUN chmod a+rwx /etc/udev/rules.d

#######################################################################
##                         install body track                        ##
#######################################################################

# RUN git clone  --recursive https://github.com/microsoft/Azure-Kinect-Samples.git && \
#     cd Azure-Kinect-Samples&& \
#     git checkout d9af2d9ed3061911fea81e88de11d5a916b23c20

# COPY /include/CMakeLists.txt /home/Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer/CMakeLists.txt

# RUN cd /home/Azure-Kinect-Samples &&\
#    mkdir -p build && \
#    cd build &&\
#    cmake .. -GNinja &&\
#    ninja

# RUN cp -r /install/libk4abt1.0/usr/bin/dnn_model_2_0.onnx  /usr/bin/

#######################################################################
##                            cleaning up                            ##
#######################################################################

RUN  rm -rf /var/lib/apt/lists/*

#######################################################################
##                         catkin setting                            ##
#######################################################################

#init catkin_ws
RUN mkdir -p /catkin_ws/src && \
   cd /catkin_ws/src && \
   /bin/bash -c 'source /opt/ros/melodic/setup.bash;catkin_init_workspace' && \
   cd /catkin_ws && \
   /bin/bash -c 'source /opt/ros/melodic/setup.bash;catkin_make'

RUN cd /catkin_ws/src && \
    git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b melodic && \
    cd Azure_Kinect_ROS_Driver && \
    git checkout cda16bc8733a3851c2c723ea433d32aa4fa5468b

RUN cd /catkin_ws && \
   /bin/bash -c 'source /opt/ros/melodic/setup.bash;catkin_make --force-cmake'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

#######################################################################
##                           ros settings                            ##
#######################################################################

RUN echo "export PS1='\[\e[1;31;40m\]AzureKinect\[\e[0m\] \u:\w\$ '">> ~/.bashrc