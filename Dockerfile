# Author: Stepan Letsko
# Date: January 10, 2026
# Purpose: This Dockerfile builds a reproducible environment for the Thesis project.
#          It sets up ROS 2 Humble, installs necessary dependencies for FAST_LIO and Livox drivers,
#          and prepares the workspace for building and running the SLAM system.

# Start from the official ROS 2 Humble base image (contains minimal ROS installation)
FROM ros:humble-ros-base

# 1. Install basic tools and dependencies for FAST_LIO
# Update package lists and install packages with automatic 'yes' to prompts
RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates gnupg && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y --fix-missing \
    git \
    # git: Version control system to clone repositories
    cmake \
    # cmake: Cross-platform build system generator used by ROS packages
    build-essential \
    # build-essential: Contains GCC/G++ compilers and Make tool
    libpcl-dev \
    # libpcl-dev: Point Cloud Library development files (headers/libs) for 3D processing
    ros-humble-pcl-ros \
    # ros-humble-pcl-ros: ROS 2 interface for PCL, allows PCL types in ROS nodes
    ros-humble-pcl-conversions \
    # ros-humble-pcl-conversions: Utilities to convert between ROS PointCloud2 messages and PCL objects
    ros-humble-libpointmatcher \
    # ros-humble-libpointmatcher: Library for Iterative Closest Point (ICP) matching
    ros-humble-rviz2 \
    # ros-humble-rviz2: 3D visualization tool for ROS 2 (to see the map and lidar data)
    ros-humble-velodyne \
    # GTSAM for Pose Graph Optimization
    ros-humble-gtsam \
    # Foxglove Bridge for visualization
    ros-humble-foxglove-bridge \
    # Tools to download missing headers
    wget \
    libboost-all-dev \
    libyaml-cpp-dev \
    libpcap-dev \
    # Network tools for debugging (ping, ifconfig)
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*
    # Clean up the apt cache to keep the Docker image size small

# Download missing ScanContext headers directly into the source tree (if mounting volume, this might be hidden, so we do it in a temp spot or assume user does it. 
# BETTER: Add them to the repo. But for now, let's install them to a system include path so they are always found.)
RUN wget https://raw.githubusercontent.com/jlblancoc/nanoflann/master/include/nanoflann.hpp -O /usr/local/include/nanoflann.hpp && \
    wget https://raw.githubusercontent.com/gisbi-kim/scancontext/main/cpp/module/Scancontext/KDTreeVectorOfVectorsAdaptor.h -O /usr/local/include/KDTreeVectorOfVectorsAdaptor.h

# 2. Install Livox-SDK2 (Required by livox_ros_driver2)
# We clone and install it into the system folders of the container manually
# because it is not available as a standard apt package.
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    # Clone the SDK source code to a temporary directory
    cd /tmp/Livox-SDK2 && \
    # Enter the directory
    mkdir build && cd build && \
    # Create a build directory and enter it (standard CMake practice)
    cmake .. && make -j$(nproc) && make install && \
    # Configure with CMake, compile using all available CPU cores, and install to system paths
    rm -rf /tmp/Livox-SDK2
    # Remove the source code to save space after installation

# 3. Setup the environment
# Set the default working directory inside the container to the ROS workspace root
WORKDIR /root/ros2_ws

# Add the ROS 2 setup script to .bashrc so it is sourced automatically
# whenever a new terminal is opened inside the container.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo 'export CPATH=$CPATH:$(python3 -c "import numpy; print(numpy.get_include())")' >> ~/.bashrc

# 4. Install Colcon (Build tool)
# Update apt again (good practice before new install block) and install colcon
# python3-colcon-common-extensions: The standard build tool for ROS 2 workspaces
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    && pip3 install "rosbags" "numpy<2.0" lz4 pandas matplotlib psutil open3d

# 5. Install SBG ROS2 Driver
# Clone source, install dependencies (fixing the libapr1-dev issue), and build
RUN mkdir -p src && \
    git clone --recursive https://github.com/SBG-Systems/sbg_ros2_driver.git src/sbg_ros2_driver && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-path src --ignore-src -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select sbg_driver --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# 6. Install Hesai ROS2 Driver
# Clone source, install dependencies, and build
RUN git clone --recursive https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git src/HesaiLidar_ROS_2.0 && \
    apt-get update && \
    rosdep install --from-path src --ignore-src -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select hesai_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# 7. Install evo for trajectory evaluation
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install evo --upgrade --no-binary evo && \
    rm -rf /var/lib/apt/lists/*

# 8. Setup Entrypoint
# Copy the entrypoint script into the image
COPY entrypoint.sh /entrypoint.sh
# Make it executable
RUN chmod +x /entrypoint.sh
# Set the entrypoint to run our script on container startup
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
