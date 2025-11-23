.PHONY: build
build:
	cd ~/ros2_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

.PHONY: ursim
ursim:
	ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=false