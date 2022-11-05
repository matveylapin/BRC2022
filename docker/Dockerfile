ARG base_img

FROM ${base_img}

# Install useful packages

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
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-turtlebot3*

# Install additional ROS packages

RUN apt-get install -y \
    ros-galactic-pointcloud-to-laserscan

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
