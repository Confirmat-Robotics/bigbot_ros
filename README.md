# README #

This repository bigbot_ros contains the building blocks to start using ROS2 on bigbot. In the [For Developers](https://confirmatrobotics.com/for-developers/) section more information can be found about the hardware integration. 

### What is this repository for? ###

* Baic ROS2 integration on the bigbot
* Version 1
* This software will read the RC receiver on the bigbot and steer the bigbot accordingingly. Also a rosbag can be started or stopped, and there is (optional but recommended) feedback to a USB-speaker and three drive-modes can be set.

### What are pre-requisites? ###
* Linux based machine 
* ROS2 installed
* espeak installed (if you have a speaker, preferably an external usb-speaker you can mount on the robot)

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
```systemctl –user enable bigbot_rc.service``` and 
```systemctl –user start bigbot_rc.service```
* If you type the following you should see a status report:
```systemctl –user status bigbot_rc.service```

## Controlling the robot ##
If the software is running properly you can control the robot as described.

Controlling is done with the remote control.

| Switch | Function                             |
| -------|--------------------------------------|
| SwB	 | switch on and off rosbag recording   |
| SwC    | switch drive mode                    |
| SwD    | Safety on/off (not software related) |

### Rosbag recording ###

| SwB    | Function                        |
| -------|---------------------------------|
| Up	 | Stop recording                  |
| Down   | Start recording                 |

If you have a speaker implemented and when you start the recording than you get voice-feedback (audio) if the start of the recording was feasible or not. If you stop recording you also get voice-feedback.

### Drive modes ###
The drive-modes are:

| SwC    | Function                             |
| -------|--------------------------------------|
| Up	 | Manual full control                  |
| Center | Manual limited radius                |
| Down   | Autonomous mode (not implemented)    |

The mode 'Manual limited radius' is usefull when you are towing a load. 

AWARE: if you just started the software you start in not any drive mode. You have to switch SwC to a mode. You can only switch on the remote if the SwC is Up. To choose the 'Manual full control' than first flip the switch to Center and back to Up-position. Now you have selected the 'Manual full control'.

If you have a speaker implemented and when you change the drive mode than you get voice-feedback.

### Safety on/off ###
It is not a software feature but electronic.This is added for completeness.

| SwD    | Function                             |
| -------|--------------------------------------|
| Up	 | Holding brakes are braking. Drive disconnected from batteries. |
| Down   | Holding brakes released. Drive connected to batteries.         |

You can only switch on the remote if the SwD is Up, so when the drive is not able to power the motors. This is a safety feature.
