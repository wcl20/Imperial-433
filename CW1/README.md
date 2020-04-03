# README #

This is the AR Drone practical repository.

Assuming a Ubuntu 18.04 installation:

Install system dependencies -- this has been done for all the machines in Huxley 202/206:

    sudo apt-get install libsdl1.2-dev libsdl2-dev

Install ROS -- this has been done for all the machines in Huxley 202/206:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-melodic-desktop-full
    
Setup by user -- see practical 1 task sheet.
    
