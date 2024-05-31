# If you want to use noetic with ubuntu 20.04, please replace melodic to noetic
FROM osrf/ros:melodic-desktop-full
# FROM osrf/ros:noetic-desktop-full

WORKDIR /home
# Since the base image is modified by its author, update the package list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-snapshots.list'
# If terminal report problems about GPG error, please replace F42ED6FBAB17C654 to your key
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update
RUN apt-get install git -y


WORKDIR /home/slamesh/lib
# # Install Ceres Solver 2.1.0
RUN apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev -y
RUN git clone https://github.com/ceres-solver/ceres-solver.git -b 2.1.0
WORKDIR /home/slamesh/lib/ceres-solver
RUN mkdir ceres-bin && cd ceres-bin && \
    cmake .. && make -j$(($(nproc)-2)) && \
    make install

# Install lvr2
WORKDIR /home/slamesh/lib
RUN apt-get install build-essential \
        cmake cmake-curses-gui libflann-dev \
        libgsl-dev libeigen3-dev libopenmpi-dev \
        openmpi-bin opencl-c-headers ocl-icd-opencl-dev \
        libboost-all-dev \
        freeglut3-dev libhdf5-dev qtbase5-dev \
        qt5-default libqt5opengl5-dev liblz4-dev \
        libopencv-dev libyaml-cpp-dev -y
# If you want to use noetic with ubuntu 20.04, please replace libvtk6 to libvtk7
RUN apt-get install libvtk6-dev libvtk6-qt-dev -y
# RUN apt-get install libvtk7-dev libvtk7-qt-dev -y

RUN git clone https://github.com/uos/lvr2.git
WORKDIR /home/slamesh/lib/lvr2
RUN mkdir build && cd build && \
    cmake .. && make -j$(($(nproc)-2)) && \
    make install
# Since slamesh cannot find the lvr2 library, copy the library to /usr/lib
RUN cp -r /home/slamesh/lib/lvr2/build/lib/* /usr/lib/


WORKDIR /home/slamesh/slamesh_ws/src
# Install mesh_tools
RUN git clone https://github.com/naturerobots/mesh_tools.git -b noetic
WORKDIR /home/slamesh/slamesh_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Install SLAMesh
WORKDIR /home/slamesh/slamesh_ws/src
RUN git clone https://github.com/RuanJY/SLAMesh.git


# Build it!!
WORKDIR /home/slamesh/slamesh_ws
# If you want to use noetic with ubuntu 20.04, please replace melodic to noetic
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -j$(($(nproc)-2))"
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j$(($(nproc)-2))"
