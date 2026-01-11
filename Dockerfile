FROM ros:jazzy-ros-base

RUN apt-get update && apt-get upgrade -y && \
	apt-get install -y \
	git \
	vim \
	tree \
	less \
	fzf \
	tmux \
	fish \
	lsof \
	usbutils\
	ccache \
	python3-pip \
	python3-gpiozero \
	libboost-system-dev \
	ros-jazzy-diagnostic-updater\
	ros-jazzy-laser-proc\
	ros-jazzy-slam-toolbox\
	ros-jazzy-rviz2\
  ros-jazzy-joy \
  ros-jazzy-demo-nodes-cpp && \
	rm -rf /var/lib/apt/lists/* # Clean up apt cache


WORKDIR /root/ros_ws

# 先にCOPYUしないとsrcがない！
COPY ./ros_ws/src src

RUN rosdep update && \
	rosdep install -i \
	--from-paths src \
	--ignore-src \
	--rosdistro jazzy -y
	



# Ccache
ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G



RUN mkdir -p /root/.config/colcon && \
    echo 'build:' > /root/.config/colcon/defaults.yaml && \
    echo '  args: ['\''--symlink-install'\'']' >> /root/.config/colcon/defaults.yaml && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.profile



CMD ["bash"]
