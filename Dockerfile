
# arm64でしか動かない
FROM ros:jazzy-ros-base 

WORKDIR /root/ros_ws

COPY ./ros_ws/src ./src

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
	usbutils \
	ccache \
	python3-pip \
	python3-gpiozero \
	libboost-system-dev \
	ros-jazzy-slam-toolbox \
	ros-jazzy-rviz2 \
    	ros-jazzy-joy \
    	ros-jazzy-demo-nodes-cpp && \
	rm -rf /var/lib/apt/lists/* # Clean up apt cache

RUN apt-get update -y && \
	sudo rosdep fix-permissions && \
	rosdep update && \
	rosdep install -y -i \
	--from-paths src \
	--ignore-src \
	--rosdistro jazzy \
	--skip-keys "cmake_modules"



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
