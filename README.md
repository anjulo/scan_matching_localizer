# scan_matching_localizer


This is a ROS package to implement a scan matching(Registration) algorithm using PCL library from inputs of a laser scan and an already-built occupancy grid map. 

We have used map_server package to read a 2-D occupancy grid map from disk using map_server node. In addition, to save an occupancy grid map to a disk we use map_saver node of the same package. This way we can deal with pre-built maps.

It changes a laser scan data from LiDAR sensor and 2-D occupancy grid map into a point-cloud data structure to manipulate the data easily using the PCL library. 
It them implements scan matching(registration) using the ICP and NDT algorithms of PCL library. By doing so, it gets the translation and rotation(rigid transform) that minimizes the distance error between the incoming scan points and pre-built map. In addition, it also has filter to make the scan matching processes easier.

The package contains bag file of scan data(laser data) and map file which can be loaded using the ROS [map_server](http://wiki.ros.org/map_server) package. The .bag file is from [KAIST Unmanned System Research Group](unmanned.kaist.ac.kr)

In addition, You should clone a git ndt_omp package that supports NDT multi-threads computation. It should be put in the catkin_ws/src directory. 
link: https://github.com/koide3/ndt_omp 



