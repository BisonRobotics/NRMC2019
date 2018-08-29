# NRMC2019
Repository for the Bison Robotics NASA Robotic Mining Competition 2019 Entry

# Getting Started

## Setup your ssh keys
Follow these instructions: https://help.github.com/articles/connecting-to-github-with-ssh/

## Clone the repo
There are certain scripts that assume that you have the NRMC2019 repo installed in your home directory. It is recommended you don't try to clone the repo anywhere else.
```
cd ~
git clone git@github.com:BisonRobotics/NRMC2019.git
```

## Configure your OS
Much of the setup process was automated last year. Assuming you are running Ubuntu 16.04, you should be able to just run the following lines of code.
```
# Go to NRMC2019/src/utilities/ansible
sudo apt update
sudo apt install ansible
sudo ansible-playbook -i "localhost," -c local dev_computer_playbook.yml
Restart your computer 


# Go to NRMC2019
catkin_make
```
If this process doesn't work for you, the manual process is described [here](https://github.com/BisonRobotics/NRMC2019/wiki/Manual-Configuration)

## Setup an IDE
I highly recommend using the clion IDE available at https://www.jetbrains.com/clion/. Make sure you sign up for an education account so you don't have to pay for it. Otherwise if you're as hardcore as Dr. Ding, VIM works too. 

In order for clion to see the libraries available to ROS, you'll need to source your ROS install setup.bash and ROS workspace setup.bash (after a build) before launching clion from the same terminal

## Setup Workspace
Our workspace is the NRM2019 repository this README lives in. You can add some optional aliases that make launching stuff and working with ROS easier
```
echo "alias clion='~/clion/bin/clion.sh'"             >> ~/.bashrc
echo "alias ws='cd ~/NRMC2019'"                       >> ~/.bashrc
echo "alias wss='source ~/NRMC2019/devel/setup.bash'" >> ~/.bashrc
```


## Permissions issues 
If you are having permissions issue with git follow these [instructions](https://askubuntu.com/questions/858569/git-permissions-issue) 


## Understand how ROS works
Do the beginner tutorials: http://wiki.ros.org/ROS/Tutorials.

# Useful unit test commands for debugging
## Build and run single test suite
Tab complete is your friend here

catkin_make run_tests_wheel_control_gtest_test_WaypointControllerHelper2
```
# Build test for debugging 
catkin_make -DCMAKE_BUILD_TYPE=Debug test_WaypointControllerHelper2
```
## Debug the unit test
gdb ./devel/lib/wheel_control/test_WaypointControllerHelper2
```


## Known Issues/Bugs in "Not-Our-Stuff"
# CANables
You may not be able to enumerate two CANables on the same computer easily and as you would imagine. There is a bug in the CANable firmware which makes one CANable enumerate as two. So, to work around this issue: make one of them can1 and the other can2. Forget about can0. Less than nothing, it decieved you, and so should be scorned.

sudo ip link set can1 type can bitrate 500000 triple-sampling on
sudo ip link set can1 up

sudo ip link set can2 type can bitrate 500000 triple-sampling on
sudo ip link set can2 up

