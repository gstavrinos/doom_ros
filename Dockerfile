# ===================================================================
#                         STAGE 1: BUILD IMAGE
# ===================================================================
FROM ros:jazzy-ros-base AS builder

# Install required build tools and dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
        python3-rosdep \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies in a temporary directory
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements.txt --root /tmp/python-deps && \
    rm -f /tmp/requirements.txt

# Setup ROS dependencies
COPY package.xml /tmp/package.xml
RUN rosdep update && \
    rosdep install --from-paths /tmp --ignore-src -y --rosdistro jazzy --skip-keys "cydoomgeneric" && \
    rm -f /tmp/package.xml

# Copy workspace and build the package
WORKDIR /workspace
COPY . .
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build


# ===================================================================
#                      STAGE 2: RUNTIME IMAGE
# ===================================================================
FROM ros:jazzy-ros-core

# Install only the required runtime dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-joy && \
    rm -rf /var/lib/apt/lists/*

# Copy built workspace from the builder stage
WORKDIR /workspace
COPY --from=builder /workspace/install /workspace/install

# Copy installed Python dependencies from the builder stage
COPY --from=builder /tmp/python-deps/usr/local /usr/local

# Ensure Python can locate the copied dependencies
ENV PYTHONPATH="/usr/local/lib/python3.12/site-packages:$PYTHONPATH"

# Default command to launch the application
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch doom_ros doom_ros.launch.py"]
