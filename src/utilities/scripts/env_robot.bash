#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nrmc/NRMC2019/env.sh
export ON_ROBOT=1

exec "$@"