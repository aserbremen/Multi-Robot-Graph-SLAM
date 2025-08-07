# This Dockerfile is used to build a Docker image with the local state of the Multi-Robot Graph SLAM repository.

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
# build the Dockerfile from the workspace root by specifying the path to the docker file
# docker build -t some_tag/mrg_slam -f docker/jazzy_local/Dockerfile .
COPY ./src/small_gicp /home/ubuntu/ros2_ws/src/small_gicp
COPY ./src/mrg_slam /home/ubuntu/ros2_ws/src/mrg_slam
COPY ./src/mrg_slam_msgs /home/ubuntu/ros2_ws/src/mrg_slam_msgs
COPY ./src/fast_gicp /home/ubuntu/ros2_ws/src/fast_gicp
COPY ./src/ndt_omp /home/ubuntu/ros2_ws/src/ndt_omp

WORKDIR /home/ubuntu/ros2_ws

# Build all except sim packages
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --packages-select small_gicp mrg_slam mrg_slam_msgs fast_gicp ndt_omp

# Source the ROS2 and workspace environment for convenience
RUN echo "source /ros_entrypoint.sh" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

# Source the ROS2 environment
CMD ["/bin/bash"]
