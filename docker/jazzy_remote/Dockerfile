# This Dockerfile is used to build a Docker image with the remote state of the Multi-Robot Graph SLAM repository. 
# The remote state is defined in the mrg_slam.repos file, which is downloaded from the main branch of the Multi-Robot Graph SLAM repository.

FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-nmea-msgs \
    ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geodesy ros-jazzy-libg2o ros-jazzy-interactive-markers wget \
    libceres-dev ros-jazzy-rmw-zenoh-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Change the UID of the user 'ubuntu' to 1001, it is the default user of the ubuntu 24.04 image I suppose. We want our ros_user to have UID 1000
RUN usermod -u 1001 ubuntu && groupmod -g 1001 ubuntu

# Create ros_user with UID 1000
RUN useradd -u 1000 -ms /bin/bash ros_user

# Update file ownership for both users
RUN find / -user 1000 -exec chown -h ubuntu:ubuntu {} \; || true
RUN find / -user 1001 -exec chown -h ros_user:ros_user {} \; || true

# Set the user as the host user
USER ros_user

RUN mkdir -p /home/ros_user/ros2_ws/src
WORKDIR /home/ros_user/ros2_ws
RUN wget -O mrg_slam.repos https://raw.githubusercontent.com/aserbremen/Multi-Robot-Graph-SLAM/main/mrg_slam.repos

# vcs is installed with ros:jazzy image, vcs pull as well. For some reason vcs import doesn't checkout the specified branch
RUN vcs import src < mrg_slam.repos && vcs pull src

# Build small_gicp first since it is a dependency of mrg_slam, not handled by ament
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --packages-select small_gicp && \
    . /home/ros_user/ros2_ws/install/setup.sh && colcon build --symlink-install --packages-select mrg_slam mrg_slam_msgs fast_gicp ndt_omp

# RUN the SLAM node
ENV MODEL_NAMESPACE=atlas
ENV X=0.0
ENV Y=0.0
ENV Z=0.0
ENV ROLL=0.0
ENV PITCH=0.0
ENV YAW=0.0
ENV INIT_POSE_TOPIC="NONE"
ENV INIT_ODOM_TOPIC="NONE"
ENV USE_SIM_TIME=true
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.sh && \
    source /home/ros_user/ros2_ws/install/setup.sh && \
    ros2 launch mrg_slam mrg_slam.launch.py model_namespace:=$MODEL_NAMESPACE \
    x:=${X:-0.0} y:=${Y:-0.0} z:=${Z:-0.0} roll:=${ROLL:-0.0} pitch:=${PITCH:-0.0} yaw:=${YAW:-0.0} \
    use_sim_time:=$USE_SIM_TIME init_odom_topic:=${INIT_ODOM_TOPIC:-NONE} init_pose_topic:=${INIT_POSE_TOPIC:-NONE}"]
