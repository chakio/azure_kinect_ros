 
FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04
# FROM nvidia/cuda:11.5.0-cudnn8-runtime-ubuntu18.04

RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list
ARG DEBIAN_FRONTEND=noninteractive

#######################################################################
##                    install essential packages                     ##
#######################################################################
RUN rm /etc/apt/sources.list.d/cuda.list
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list


RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    apt-utils \
    python-pip \
    build-essential \ 
    software-properties-common \
    apt-transport-https \
    zip \
    unzip \
	g++ \
	perl \
    wget \
    git \
    nautilus\
    net-tools \
    gedit \
    curl

RUN python -m pip install --upgrade pip
RUN apt-get update && apt-get install  -y python-ruamel.yaml

#######################################################################
##                       install nvidia docker                       ##
#######################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install GLX-Gears
RUN apt update && apt install -y --no-install-recommends \
    mesa-utils x11-apps

#######################################################################
##                            install ros                            ##
#######################################################################

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosdep \
    python-vcstools 

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init 
RUN rosdep update

# install ros packages
ENV ROS_DISTRO melodic
 

# install bootstrap tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full

# setup entrypoint
COPY ./include/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#######################################################################
##                       install azure kinect                        ##
#######################################################################

# Configuring the repositories
RUN curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
RUN apt-get update && apt-get install -y \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib-arm-linux-gnueabihf \
    g++-multilib-arm-linux-gnueabihf && \
   rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:ubuntu-x-swat/updates
RUN apt-get update && \
    apt-get dist-upgrade -y
RUN apt-get update && apt-get install -y \
    freeglut3 \
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
    libssl-dev \
    mesa-utils \
    x11-apps && \
    rm -rf /var/lib/apt/lists/*



RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
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