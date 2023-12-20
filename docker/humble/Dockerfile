FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-nmea-msgs \
    ros-humble-tf2-ros ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-geodesy ros-humble-libg2o ros-humble-interactive-markers wget \
    libceres-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the user as the host user
RUN useradd -ms /bin/bash humble_user
USER humble_user

RUN mkdir -p /home/humble_user/ros2_ws/src
WORKDIR /home/humble_user/ros2_ws
RUN wget -O mrg_slam.repos https://raw.githubusercontent.com/aserbremen/Multi-Robot-Graph-SLAM/main/mrg_slam.repos
# vcs is installed with ros:humble image
RUN vcs import src < mrg_slam.repos

# Build all except sim packages
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select \
    mrg_slam mrg_slam_msgs fast_gicp ndt_omp


# RUN the SLAM node
CMD /bin/bash -c "source /opt/ros/humble/setup.sh && \
    source /home/humble_user/ros2_ws/install/setup.sh && \
    ros2 launch mrg_slam mrg_slam.launch.py"
