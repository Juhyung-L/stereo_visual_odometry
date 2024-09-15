FROM osrf/ros:melodic-desktop

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# CMake
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    git \
    build-essential \
    ca-certificates \
    wget
RUN update-ca-certificates
RUN wget https://github.com/Kitware/CMake/releases/download/v3.30.0/cmake-3.30.0-linux-x86_64.sh && \
    chmod +x ./cmake-3.30.0-linux-x86_64.sh && \
    ./cmake-3.30.0-linux-x86_64.sh --prefix=/usr/local --skip-license && \
    rm ./cmake-3.30.0-linux-x86_64.sh

# Eigen3
WORKDIR  /home/dev/
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
    tar -xvzf eigen-3.4.0.tar.gz && \
    rm eigen-3.4.0.tar.gz
WORKDIR /home/dev/eigen-3.4.0/build/
RUN cmake ..
RUN make -j$(nproc)
RUN make install

# Sophus
WORKDIR /home/dev/
RUN git clone https://github.com/strasdat/Sophus.git
WORKDIR /home/dev/Sophus/build/
RUN cmake ..
RUN make -j$(nproc)
RUN make install

# Ceres Solver
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev

WORKDIR /home/dev/
RUN wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz && \
    tar -xvzf ceres-solver-2.0.0.tar.gz
WORKDIR /home/dev/ceres-solver-2.0.0/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install

# OpenCV (in Release mode)
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libopencv-dev

# personal packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    vim \
    gdb \
    gdbserver

# remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# setup bashrc settings
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

WORKDIR /home/dev_ws