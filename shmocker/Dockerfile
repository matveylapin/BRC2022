ARG from

FROM ${from}

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

ARG USERNAME=robot
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
    ros-galactic-nav2-simple-commander 
    

RUN apt-get install -y \
    pip

RUN python3 -m pip install \
    opencv-contrib-python \
    xacro

# [Optional] Set the default user. Omit if you want to keep the default as root.

USER $USERNAME

ENV WS=/home/$USERNAME/workspace

# For vs-code server

RUN mkdir -p /home/$USERNAME/.vscode-server/extensions

RUN mkdir -p /home/$USERNAME/BRC2022/BRC2022/workspace && cd /home/$USERNAME \
    && ln -s /home/$USERNAME/BRC2022/BRC2022/workspace workspace && sudo ln -s /home/$USERNAME/BRC2022/BRC2022/workspace /workspace

WORKDIR $WS

CMD ["/bin/bash","-c","echo \"source ${WS}/install/setup.bash\"  >> ~/.bashrc && echo \"source /opt/ros/galactic/setup.bash\"  >> ~/.bashrc && /bin/bash"]
