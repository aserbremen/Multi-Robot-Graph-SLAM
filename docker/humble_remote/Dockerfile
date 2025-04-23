# This Dockerfile is used to build a Docker image with the remote state of the Multi-Robot Graph SLAM repository. 
# The remote state is defined in the mrg_slam.repos file, which is downloaded from the main branch of the Multi-Robot Graph SLAM repository.

FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-nmea-msgs \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-geodesy ros-humble-libg2o ros-humble-interactive-markers wget \
    libceres-dev ros-humble-rmw-zenoh-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create ubuntu user, to have the same user name as provided by the ubuntu 24 base image
RUN useradd -ms /bin/bash ubuntu
USER ubuntu

RUN mkdir -p /home/ubuntu/ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws
RUN wget -O mrg_slam.repos https://raw.githubusercontent.com/aserbremen/Multi-Robot-Graph-SLAM/main/mrg_slam.repos

# vcs is installed with ros:humble image, vcs pull as well. For some reason vcs import doesn't checkout the specified branch
RUN vcs import src < mrg_slam.repos && vcs pull src

# Build small_gicp first since it is a dependency of mrg_slam, not handled by ament
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select small_gicp mrg_slam mrg_slam_msgs fast_gicp ndt_omp

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
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && \
    source /home/ubuntu/ros2_ws/install/setup.sh && \
    ros2 launch mrg_slam mrg_slam.launch.py model_namespace:=$MODEL_NAMESPACE \
    x:=${X:-0.0} y:=${Y:-0.0} z:=${Z:-0.0} roll:=${ROLL:-0.0} pitch:=${PITCH:-0.0} yaw:=${YAW:-0.0} \
    use_sim_time:=$USE_SIM_TIME init_odom_topic:=${INIT_ODOM_TOPIC:-NONE} init_pose_topic:=${INIT_POSE_TOPIC:-NONE}"]
