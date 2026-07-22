# Utilisation de ROS Noetic (Desktop Full)
FROM osrf/ros:noetic-desktop-full

# 1. Mise à jour et installation des outils système
RUN apt-get update && apt-get install -y \
    nvim \
    python3-pip \
    nano \
    usbutils \
    git \
    tree \
    wget \
    python3-rosdep \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-grid-map \
    ros-noetic-joy \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-plotjuggler-ros \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# 2. Installation des librairies Python requises
RUN pip3 install --no-cache-dir \
    numpy \
    pandas \
    pyserial

# 3. Mise à jour de rosdep (init est déjà fait par l'image de base)
RUN rosdep update

# 4. Création de l'espace de travail ROS
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# 5. CLONAGE DE OUSTER-ROS (Branche par défaut pour ROS 1)
RUN git clone --recursive https://github.com/ouster-lidar/ouster-ros.git /root/catkin_ws/src/ouster-ros && \
    cd ouster-ros && \
    git checkout $(git rev-list -n 1 --before="2023-10-01" HEAD) && \
    git submodule update --init --recursive

RUN git clone --depth 1 https://github.com/ANYbotics/elevation_mapping.git
RUN git clone --depth 1 https://github.com/ANYbotics/message_logger.git
RUN git clone --depth 1 https://github.com/ANYbotics/kindr.git
RUN git clone --depth 1 https://github.com/ANYbotics/kindr_ros.git

WORKDIR /root/catkin_ws

# 6. Installation automatique des dépendances ROS requises par ouster-ros
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 7. Configuration automatique de l'environnement ROS au démarrage
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc \
    echo "if [ -f ~/.bash_aliases ]; then . ~/.bash_aliases; fi" >> /home/$USERNAME/.bashrc \
    echo "alias vi=nvim" >> /home/$USERNAME/.bashrc

# 8. Commande par défaut
CMD ["bash"]
