FROM osrf/ros:jazzy-desktop

ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    vim \
    tmux 

RUN apt-get update && apt-get install -y \
    lsb-release \
    software-properties-common \
    python3-pip \
    clang \
    clang-tools \
    lldb \
    lld \
    libstdc++-12-dev \
    python3-shapely \
    python3-yaml \
    python3-requests

RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" >> /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y zenoh || true  

RUN sed -i 's/^.*systemctl daemon-reload.*$/# &/' /var/lib/dpkg/info/zenohd.postinst \
    && sed -i 's/^.*systemctl disable zenohd.*$/# &/' /var/lib/dpkg/info/zenohd.postinst

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-dev-tools \
    ros-${ROS_DISTRO}-rmf-building-map-tools \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-nlohmann-json-schema-validator-vendor \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ament-cmake-catch2 \
    ros-${ROS_DISTRO}-tf-transformations \
    # ros-${ROS_DISTRO}-ros-gz \
    # ros-${ROS_DISTRO}-rmf-building-sim-gz-plugins \
    # ros-${ROS_DISTRO}-rmf-robot-sim-gz-plugins \
    liburdfdom-dev \
    libwebsocketpp-dev \
    # ros-${ROS_DISTRO}-rmf-building-sim-common \
    # ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # ros-${ROS_DISTRO}-gazebo-ros2-control \
    # ros-${ROS_DISTRO}-ign-ros2-control\
    # ros-${ROS_DISTRO}-ign-ros2-control-demos \
    ros-${ROS_DISTRO}-rmf-building-sim-gz-plugins\
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-fastrtps \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-controller-manager\
    ros-${ROS_DISTRO}-control*

RUN pip3 install --no-cache-dir --break-system-packages --no-deps --ignore-installed \
    colcon-common-extensions \
    empy \
    pyros-genmsg \
    jinja2 \
    fastapi \
    uvicorn \
    flask-socketio \
    eclipse-zenoh==1.1.0 \
    pycdr2 \
    rosbags \
    nudged

RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils libgl1-mesa-dri libglx-mesa0 libglu1-mesa \
    libegl1 libgbm1 \
    mesa-vulkan-drivers libvulkan1 \
    xvfb xauth x11-apps \
    ros-jazzy-rmf-building-sim-gz-plugins \
    ros-jazzy-rmf-robot-sim-gz-plugins && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/rmf_ws/src
# RUN git clone --depth=1 https://github.com/open-rmf/rmf_demos.git -b 2.0.3
# RUN git clone --depth=1 https://github.com/open-rmf/free_fleet.git 
# RUN git clone --depth=1 https://github.com/ros-planning/navigation2.git -b $ROS_DISTRO 
# RUN git clone --depth=1 https://github.com/ros-controls/ros2_control.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_ros2.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_internal_msgs.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_battery.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_visualization.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_visualization_msgs.git -b $ROS_DISTRO
RUN git clone https://github.com/open-rmf/rmf_demos.git -b $ROS_DISTRO
# RUN git clone https://github.com/open-rmf/rmf_api_msgs.git 
RUN git clone https://github.com/open-rmf/pybind11_json_vendor.git -b $ROS_DISTRO 
# RUN git clone https://github.com/open-rmf/rmf_simulation.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_task.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/menge_vendor.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_traffic.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_traffic_editor.git -b $ROS_DISTRO 
RUN git clone https://github.com/ament/ament_cmake.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_utils.git -b $ROS_DISTRO 
RUN git clone https://github.com/open-rmf/rmf_api_msgs.git -b $ROS_DISTRO 
# RUN git clone --recursive https://github.com/chvmp/champ -b ros2 
# RUN git clone https://github.com/chvmp/champ_teleop -b ros2
# RUN git clone -b humble https://github.com/ros-simulation/gazebo_ros2_control.git

RUN pip3 install --break-system-packages --no-deps --ignore-installed --upgrade pip && \
pip3 uninstall --break-system-packages -y Flask Flask-SocketIO python-engineio python-socketio Werkzeug && \
pip3 install --break-system-packages Flask-SocketIO==4.3.1 python-engineio==3.13.2 python-socketio==4.6.0 Flask==2.0.3 Werkzeug==2.0.3

ENV ZENOH_VERSION=1.1.0
RUN wget -O zenoh-plugin-ros2dds.zip \
https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip \
&& unzip zenoh-plugin-ros2dds.zip \
&& rm zenoh-plugin-ros2dds.zip

WORKDIR /root/rmf_ws

RUN . /opt/ros/$ROS_DISTRO/setup.sh
#  \
#     && rosdep update \
#     && rosdep install --from-paths src --ignore-src -r -y

# RUN apt-get update && apt-get install -y ros-jazzy-ros2-control-cmake

# Add OSRF Gazebo GPG key and repository
# RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
#     curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /usr/share/keyrings/gz.gpg > /dev/null && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gz.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
#     tee /etc/apt/sources.list.d/gz-stable.list

# RUN apt-get update && apt-get install -y gz-harmonic-all

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && export CXX=g++ \
    && export CC=gcc  \
    && colcon build --symlink-install

# . /opt/ros/$ROS_DISTRO/setup.sh && export CXX=g++ && export CC=gcc && colcon build --symlink-install

# RUN chmod +x /root/rmf_ws/install/free_fleet_adapter/lib/free_fleet_adapter/fleet_adapter.py

# RUN mkdir -p /root/rmf_ws/install/free_fleet_adapter/share/free_fleet_adapter/launch \
#     && cp /root/rmf_ws/install/free_fleet_adapter/share/free_fleet_adapter/fleet_adapter.launch.xml /root/rmf_ws/install/free_fleet_adapter/share/free_fleet_adapter/launch/
    
# RUN mkdir -p /opt/ros/jazzy/share/nav2_bringup/maps/

# ENV TURTLEBOT3_MODEL=waffle
# ENV FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
# ENV FASTRTPS_SHM_PORT=0
# # ENV ROS_DOMAIN_ID=55

# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# # ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# # RUN echo 'alias kill_gazebo_server=pkill -9 gzserver' >> ~/.bashrc
# # RUN echo 'alias kill_gazebo_client=pkill -9 gzclient' >> ~/.bashrc
# # tmux kill-session -t tbt3_fleet_manager & pkill -9 gzserver & pkill -9 gzclient
# # tmux kill-session -t tbt3_single_fleet_manager & pkill -9 gzserver & pkill -9 gzclient
# RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models' >> ~/.bashrc
# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
# RUN echo "source /root/rmf_ws/install/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

WORKDIR /root/rmf_ws

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /root/rmf_ws/install/setup.bash && ros2 launch rmf_demos_gz office.launch.xml headless:=1"]
