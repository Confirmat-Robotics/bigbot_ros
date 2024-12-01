# README #

The package bigbot_description contains the urdf model of the bigbot for use of visualization of the robot in rviz, or in simulators such as Gazebo.
The image below shows a joint state publisher which allows manipulation of the wheels. 

![Urdf in rviz with joint state publisher](https://bitbucket.org/edhage/bigbot_ros/downloads/urdf_of_bigbot.png)


### Addons ###
Besides the bigbot also two addons have been modelled. 

The fixposition GTK-RTK module which allows for accurate positioning of the bigbot.

![GPS-RTK accurate positioning of the bigbot](https://bitbucket.org/edhage/bigbot_ros/downloads/fixpos_gps.png)


The TIM lidar for mapping and obstacle avoidance.

![TIM lidar](https://bitbucket.org/edhage/bigbot_ros/downloads/lidar.png)


### Gazebo simulation ###

In the directory model an sdf model is given that can be used in gazebo. For actually steering edit this file, at the end of the file give the location of your bigbot_controller controller file.

#### Sensors ####

The model provides simulation of these sensors:

* GPS in the fixposition node
* LIDAR in the TIM lidar node 

If you do not use a fixposition GPS or a TIM lidar you can manually remove these links from the urdf and regenerate the sdf using the gz-command (provided by the makers of Gazebo).

#### Controller ####

The gazebo model has a controller for the wheels. An actual speed-controller is in the package bigbot_controller.
The two left wheels are driven by one controller, and the two right wheels by another. Cornering is done by skidding.

