# Utilisation de ROS Noetic (Desktop Full)
FROM osrf/ros:noetic-desktop-full

# 1. Mise à jour et installation des outils système
RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    usbutils \
    git \
    wget \
    python3-rosdep \
    ros-noetic-tf2-sensor-msgs \
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
WORKDIR /root/catkin_ws

# 5. CLONAGE DE OUSTER-ROS (Branche par défaut pour ROS 1)
RUN git clone --recursive https://github.com/ouster-lidar/ouster-ros.git /root/catkin_ws/src/ouster-ros

# 6. Installation automatique des dépendances ROS requises par ouster-ros
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 7. Configuration automatique de l'environnement ROS au démarrage
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc

# 8. Commande par défaut
CMD ["bash"]
