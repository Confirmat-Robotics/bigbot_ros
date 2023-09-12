# README #

This repository bigbot_ros contains the building blocks to start using ROS2 on bigbot. In the [For Developers](https://confirmatrobotics.com/for-developers/) section more information can be found about the hardware integration. 

### What is this repository for? ###

* Baic ROS2 integration on the bigbot
* Version 1

### What are pre-requisites? ###
* Linux based machine 
* ROS2 installed

The software is tested on Ubuntu 20.04 with ROS2 Galactic, but should work with Humble too.

### How do I get set up? ###

* Clone this repo on the computer.
* Goto the git-directory and build it using colcon build
* Copy the file bigbot/systemd/network-wait-online.service to folder $HOME/.config/systemd/user
* Copy the file bigbot/systemd/bigbot_rc.service to folder $HOME/.config/systemd/user and edit it.
* The line 
```ExecStart=/bin/bash -c 'source /opt/ros/galactic/setup.bash; source /home/<installdirectory>/bigbot/install/setup.bash; ros2 launch bigbot_bringup rcdrive_launch.py'``` 
must be changed according to your installdirectory. Save the file.
* In a terminal type the following to enable the service:
```systemctl –user enable bigbot_rc.service```
```systemctl –user start bigbot_rc.service```
* If you type the following you should see a status report:
```systemctl –user status bigbot_rc.service```

## Using the software ##
