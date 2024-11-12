# This Dockerfile is used to build a Docker image with the local state of the Multi-Robot Graph SLAM repository.

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
# build the Dockerfile from the workspace root by specifying the path to the docker file
# docker build -t some:tag -f docker/humble_local/Dockerfile .
COPY ./src/mrg_slam /home/humble_user/ros2_ws/src/mrg_slam
COPY ./src/mrg_slam_msgs /home/humble_user/ros2_ws/src/mrg_slam_msgs
COPY ./src/fast_gicp /home/humble_user/ros2_ws/src/fast_gicp
COPY ./src/ndt_omp /home/humble_user/ros2_ws/src/ndt_omp

WORKDIR /home/humble_user/ros2_ws

# Build all except sim packages
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select \
    mrg_slam mrg_slam_msgs fast_gicp ndt_omp

# RUN the SLAM node
ENV MODEL_NAMESPACE=atlas
ENV X=0.0
ENV Y=0.0
ENV Z=0.0
# degrees
ENV ROLL=0.0
ENV PITCH=0.0
ENV YAW=0.0
ENV INIT_POSE_TOPIC="NONE"
ENV INIT_ODOM_TOPIC="NONE"
ENV USE_SIM_TIME=true
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && \
    source /home/humble_user/ros2_ws/install/setup.sh && \
    ros2 launch mrg_slam mrg_slam.launch.py model_namespace:=$MODEL_NAMESPACE \
    x:=$X y:=$Y z:=$Z roll:=$ROLL pitch:=$PITCH yaw:=$YAW \
    use_sim_time:=$USE_SIM_TIME init_odom_topic:=${INIT_ODOM_TOPIC:-NONE} init_pose_topic:=${INIT_POSE_TOPIC:-NONE}"]
