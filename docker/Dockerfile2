
ARG base_img
FROM ${base_img}

#
# setup environment
#
ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

WORKDIR /opt


#
# OpenCV - https://github.com/mdegans/nano_build_opencv/blob/master/build_opencv.sh
#
ARG OPENCV_VERSION="4.4.0"

# install build dependencies
COPY scripts/opencv_install_deps.sh opencv_install_deps.sh
RUN ./opencv_install_deps.sh

# OpenCV looks for the cuDNN version in cudnn_version.h, but it's been renamed to cudnn_version_v8.h
RUN ln -s /usr/include/$(uname -i)-linux-gnu/cudnn_version_v8.h /usr/include/$(uname -i)-linux-gnu/cudnn_version.h

# architecture-specific build options
ARG CUDA_ARCH_BIN=""
ARG ENABLE_NEON="OFF"

# clone and configure OpenCV repo
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    echo "configuring OpenCV ${OPENCV_VERSION}, CUDA_ARCH_BIN=${CUDA_ARCH_BIN}, ENABLE_NEON=${ENABLE_NEON}" && \
    cmake \
        -D CPACK_BINARY_DEB=ON \
	   -D BUILD_EXAMPLES=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
	   -D BUILD_opencv_java=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
        -D CUDA_ARCH_PTX= \
        -D CUDA_FAST_MATH=ON \
        -D CUDNN_INCLUDE_DIR=/usr/include/$(uname -i)-linux-gnu \
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
	   -D WITH_EIGEN=ON \
        -D ENABLE_NEON=${ENABLE_NEON} \
        -D OPENCV_DNN_CUDA=ON \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_OPENGL=ON \
	   -D WITH_OPENCL=OFF \
	   -D WITH_IPP=OFF \
        -D WITH_TBB=ON \
	   -D BUILD_TIFF=ON \
	   -D BUILD_PERF_TESTS=OFF \
	   -D BUILD_TESTS=OFF \
	   ../
	   
RUN cd opencv/build && make -j$(nproc)
RUN cd opencv/build && make install
RUN cd opencv/build && make package

RUN cd opencv/build && tar -czvf OpenCV-${OPENCV_VERSION}-$(uname -i).tar.gz *.deb

RUN apt-get update && apt-get install -y \
    sudo \
    apt-utils \
    mesa-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim \
    nano

# Create non-root user

ARG USERNAME=tolya
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install navigation ROS packages

RUN apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Install additional ROS packages

RUN apt-get install -y \
    ros-humble-pointcloud-to-laserscan

# opencv-python

RUN apt-get install -y \
    pip

RUN python3 -m pip install opencv-contrib-python

RUN apt install -y tmux

RUN usermod -a -G dialout $USERNAME

# SSH server setup

RUN apt-get update && apt-get install -y openssh-server

RUN echo 'tolya:sttb' | chpasswd && \
    echo "Port 23" >> /etc/ssh/sshd_config && \
    echo "ForceCommand /bin/bash" >> /etc/ssh/sshd_config

# [Optional] Set the default user. Omit if you want to keep the default as root.

USER $USERNAME

ENV WS=/home/$USERNAME/tolya/workspace

# For vs-code server

RUN mkdir -p /home/$USERNAME/.vscode-server/extensions

WORKDIR /home/$USERNAME/tolya

CMD ["echo \"source ${WS}/install/setup.bash\"  >> ~/.bashrc && \
    sudo service ssh start && \
    /bin/bash"]
