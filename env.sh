#!/usr/bin/env bash
if [ -d "/opt/ros/kinetic" ]; then
	source /opt/ros/kinetic/setup.bash
fi
#if [ -d "/opt/ros/melodic" ]; then
	#source /opt/ros/melodic/setup.bash
#fi
source /home/$(whoami)/NRMC2019/devel/setup.bash
export ROSCONSOLE_FORMAT='[${severity}]: ${message}'
export ROS_MASTER_URI=http://up2:1234
exec "$@"
