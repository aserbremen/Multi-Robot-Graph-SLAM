# This Dockerfile is used to build a Docker image with the local state of the Multi-Robot Graph SLAM repository.

FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-nmea-msgs \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-geodesy ros-humble-libg2o ros-humble-interactive-markers wget \
    libceres-dev ros-humble-rmw-zenoh-cpp ros-humble-rcutils libgeographic-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create ubuntu user, to have the same user name as provided by the ubuntu 24 base image
RUN useradd -ms /bin/bash ubuntu
USER ubuntu

RUN mkdir -p /home/ubuntu/ros2_ws/src
# build the Dockerfile from the workspace root by specifying the path to the docker file
# docker build -t some_tag/mrg_slam -f docker/humble_local/Dockerfile .
COPY ./src/small_gicp /home/ubuntu/ros2_ws/src/small_gicp
COPY ./src/mrg_slam /home/ubuntu/ros2_ws/src/mrg_slam
COPY ./src/mrg_slam_msgs /home/ubuntu/ros2_ws/src/mrg_slam_msgs
COPY ./src/fast_gicp /home/ubuntu/ros2_ws/src/fast_gicp
COPY ./src/ndt_omp /home/ubuntu/ros2_ws/src/ndt_omp

WORKDIR /home/ubuntu/ros2_ws

# Build all except sim packages
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select small_gicp mrg_slam mrg_slam_msgs fast_gicp ndt_omp

# Source the ROS2 and workspace environment for convenience
RUN echo "source /ros_entrypoint.sh" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.sh" >> /home/ubuntu/.bashrc

# Source the ROS2 environment
CMD ["/bin/bash"]
