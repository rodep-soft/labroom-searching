
setup:
	git config merge.ours.driver true

mark:
	apt-mark showmanual >> packages.txt
	sort -u packages.txt -o packages.txt

sync:
	sudo xargs apt install -y < packages.txt

alter:
	sed -i 's/ros:jazzy-ros-base/osrf\/ros:jazzy-desktop-full/g' Dockerfile

restore:
	sed -i 's/\/ros:jazzy-desktop-full/ros:jazzy-ros-base/g' Dockerfile
