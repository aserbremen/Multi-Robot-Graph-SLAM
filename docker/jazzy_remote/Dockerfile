# This Dockerfile is used to build a Docker image with the remote state of the Multi-Robot Graph SLAM repository. 
# The remote state is defined in the mrg_slam.repos file, which is downloaded from the main branch of the Multi-Robot Graph SLAM repository.

FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-nmea-msgs \
    ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geodesy ros-jazzy-libg2o ros-jazzy-interactive-markers wget \
    libceres-dev ros-jazzy-rmw-zenoh-cpp libgeographiclib-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the user that is created in the ubuntu base image
USER ubuntu

RUN mkdir -p /home/ubuntu/ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws
RUN wget -O mrg_slam.repos https://raw.githubusercontent.com/aserbremen/Multi-Robot-Graph-SLAM/main/mrg_slam.repos

# vcs is installed with ros:jazzy image, vcs pull as well. For some reason vcs import doesn't checkout the specified branch
RUN vcs import src < mrg_slam.repos && vcs pull src

# Build small_gicp first since it is a dependency of mrg_slam, not handled by ament
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --packages-select small_gicp mrg_slam mrg_slam_msgs fast_gicp ndt_omp

# Source the ROS2 and workspace environment for convenience
RUN echo "source /ros_entrypoint.sh" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.sh" >> /home/ubuntu/.bashrc

# Source the ROS2 environment
CMD ["/bin/bash"]
