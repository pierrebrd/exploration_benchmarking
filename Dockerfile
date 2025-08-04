FROM ros:humble

# Install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    wget \
    xvfb \
    wmctrl \
    x11-utils \
    libfltk1.3-dev \
    ros-humble-ament-cmake \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*


# Install Python dependencies
RUN pip3 install pip --upgrade
RUN pip3 install --no-cache-dir --break-system-packages \
    matplotlib \
    evo 

# Copy source code to workspace
ENV WORKDIR=/root/exploration_benchmarking
RUN mkdir -p ${WORKDIR}
COPY . ${WORKDIR}

# Set up ROS workspace
ENV ROS_WS=/root/exploration_benchmarking/ros2_simulation
WORKDIR ${ROS_WS}
RUN rm -rf build log install

# probably a lot of things missing and other things not required


# Install ROS 2 dependencies
RUN [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update


RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt update

RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select openslam_gmapping"
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source ${ROS_WS}/install/setup.bash && colcon build --symlink-install"
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

ENV DISPLAY=:1
ENV screen=0
ENV resolution=800x600x24
RUN echo "Xvfb ${DISPLAY} -screen ${screen} ${resolution} &" >> ~/.bashrc

# COPY launch.sh /root/launch.sh
# RUN chmod +x /root/launch.sh

# ENTRYPOINT ["/bin/bash", "-i", "/root/launch.sh"]

CMD ["bash"]
