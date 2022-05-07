FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list
ARG DEBIAN_FRONTEND=noninteractive

#######################################################################
##                    install essential packages                     ##
#######################################################################

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
   && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip
RUN apt-get update && apt-get install  -y python-ruamel.yaml && \
   rm -rf /var/lib/apt/lists/*

#######################################################################
##                       install nvidia docker                       ##
#######################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install GLX-Gears
RUN apt update && apt install -y --no-install-recommends \
    mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*

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

# install bootstrap tools
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full

RUN apt-get update && apt-get install -y \
    curl

# install ros packages
ENV ROS_DISTRO melodic
 
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
    g++-multilib-arm-linux-gnueabihf

RUN apt-get install -y \
    freeglut3-dev \
    libgl1-mesa-dev \
    mesa-common-dev \
    libsoundio-dev \
    libvulkan-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    uuid-dev \
    libsdl2-dev \
    usbutils \
    libusb-1.0-0-dev \
    openssl \
    libssl-dev
# update cmake
RUN wget https://cmake.org/files/v3.16/cmake-3.16.5.tar.gz  -O cmake-3.16.5.tar.gz
RUN tar -zxvf cmake-3.16.5.tar.gz 
WORKDIR /cmake-3.16.5

RUN ./bootstrap
RUN make
RUN make install

# install azure kinect sdk
WORKDIR /
RUN wget https://www.nuget.org/api/v2/package/Microsoft.Azure.Kinect.Sensor/1.3.0 -O microsoft.azure.kinect.sensor.1.3.0.nupkg 
RUN mv microsoft.azure.kinect.sensor.1.3.0.nupkg  microsoft.azure.kinect.sensor.1.3.0.zip
RUN unzip -d microsoft.azure.kinect.sensor.1.3.0 microsoft.azure.kinect.sensor.1.3.0.zip

WORKDIR /home

RUN git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git -b release/1.3.x
RUN cd /home/Azure-Kinect-Sensor-SDK &&\
    git branch -a
RUN mkdir -p /home/Azure-Kinect-Sensor-SDK/build/bin/
RUN cp /microsoft.azure.kinect.sensor.1.3.0/linux/lib/native/x64/release/libdepthengine.so.2.0 /home/Azure-Kinect-Sensor-SDK/build/bin/libdepthengine.so.2.0
RUN cp /microsoft.azure.kinect.sensor.1.3.0/linux/lib/native/x64/release/libdepthengine.so.2.0 /lib/x86_64-linux-gnu/
RUN cp /microsoft.azure.kinect.sensor.1.3.0/linux/lib/native/x64/release/libdepthengine.so.2.0 /usr/lib/x86_64-linux-gnu/
RUN chmod a+rwx /usr/lib/x86_64-linux-gnu
RUN chmod a+rwx -R /lib/x86_64-linux-gnu/
RUN chmod a+rwx -R /home/Azure-Kinect-Sensor-SDK/build/bin/

RUN cd /home/Azure-Kinect-Sensor-SDK &&\
    mkdir -p build && \
    cd build &&\
    cmake .. -GNinja &&\
    ninja &&\
    ninja install

RUN mkdir -p /etc/udev/rules.d/
RUN cp /home/Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/99-k4a.rules
RUN chmod a+rwx /etc/udev/rules.d

#######################################################################
##                         install body track                        ##
#######################################################################

RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb
RUN mkdir -p /install
RUN dpkg -x ./libk4abt1.0-dev_1.0.0_amd64.deb /install/libk4abt1.0
RUN dpkg -x ./libk4abt1.0_1.0.0_amd64.deb /install/libk4abt1.0
RUN apt-get update && apt-get install -y \
    libxi-dev
    
COPY /include/k4abtConfig.cmake /install/libk4abt1.0/usr/lib/cmake/k4abt/k4abtConfig.cmake 

RUN git clone --recursive https://github.com/microsoft/Azure-Kinect-Samples.git

RUN cp /install/libk4abt1.0/usr/lib/libk4abt.so /lib/x86_64-linux-gnu/
RUN cp /install/libk4abt1.0/usr/lib/libk4abt.so /usr/lib/x86_64-linux-gnu/
RUN cp /install/libk4abt1.0/usr/lib/libk4abt.so.1.0.0 /lib/x86_64-linux-gnu/
RUN cp /install/libk4abt1.0/usr/lib/libk4abt.so.1.0.0 /usr/lib/x86_64-linux-gnu/
#
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so /lib/x86_64-linux-gnu/
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so /usr/lib/x86_64-linux-gnu/
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so.1.3 /lib/x86_64-linux-gnu/
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so.1.3 /usr/lib/x86_64-linux-gnu/
RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so.1.3.0 /lib/x86_64-linux-gnu/
RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so.1.3.0 /usr/lib/x86_64-linux-gnu/
#
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4arecord.so /lib/x86_64-linux-gnu/
# RUN cp /home/Azure-Kinect-Sensor-SDK/build/bin/libk4arecord.so /usr/lib/x86_64-linux-gnu/

RUN cp /install/libk4abt1.0/usr/include/k4abt.h /usr/include/ 
RUN cp /install/libk4abt1.0/usr/include/k4abt.hpp /usr/include/
RUN cp /install/libk4abt1.0/usr/include/k4abttypes.h /usr/include/
RUN cp /install/libk4abt1.0/usr/include/k4abtversion.h /usr/include/

RUN cp -r  /home/Azure-Kinect-Sensor-SDK/include/k4a/ /usr/include/
RUN cp -r /home/Azure-Kinect-Sensor-SDK/include/k4ainternal/ /usr/include/
RUN cp -r /home/Azure-Kinect-Sensor-SDK/include/k4arecord/ /usr/include/
RUN cp -r /home/Azure-Kinect-Sensor-SDK/include/k4arecord/ /usr/include/

COPY /include/CMakeLists.txt /home/Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer/CMakeLists.txt

ENV PATH=${PATH}:/install/libk4abt1.0/usr/:/install/libk4abt1.0/lib/
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/install/libk4abt1.0/usr/lib/:/home/Azure-Kinect-Sensor-SDK/build/bin/
# RUN cd /home/Azure-Kinect-Samples &&\
#    mkdir -p build && \
#    cd build &&\
#    cmake .. -GNinja &&\
#    ninja

RUN cp -r /install/libk4abt1.0/usr/bin/dnn_model_2_0.onnx  /usr/bin/


#######################################################################
##                     install optional packages                     ##
#######################################################################

RUN apt-get update && apt-get install -y \
    nautilus\
    net-tools \
    gedit

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
    git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git

RUN cd /catkin_ws && \
   /bin/bash -c 'source /opt/ros/melodic/setup.bash;catkin_make --force-cmake'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc


#######################################################################
##                           ros settings                            ##
#######################################################################

RUN echo "export PS1='\[\e[1;31;40m\]AzureKinect\[\e[0m\] \u:\w\$ '">> ~/.bashrc


