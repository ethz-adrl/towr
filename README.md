### ![](https://i.imgur.com/ct8e7T4h.png)  

| ![](https://i.imgur.com/NkL8Haw.gif) | ![](https://i.imgur.com/RrEc2Cd.gif) 
|:-------------------------:|:-------------------------:|
|||

xpp_opt formulates the Trajectory Optimization Problem for floating base systems and solves it using IPOPT or SNOPT. The solutions are given as [xpp] bags and can be visualized using (http://wiki.ros.org/xpp).

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)** 

This code was developed at the [Agile and Dexterous Robotics Lab](http://www.adrl.ethz.ch/doku.php), ETH Zurich. It is currently improved at the [Robotics Systems Lab](http://www.rsl.ethz.ch/), ETH Zurich.


## Packages

  * opt_solve: A generic wrapper for NLP solvers [Ipopt]/[Snopt] depended only on [Eigen].
  * xpp_opt: The Trajectory Optimization formulation for floating base systems (ros independent).
  * xpp_opt_ros: A ros wrapper with interactive keyboard input and sending out the optimized motions to be visualized with [xpp].

## Dependencies

[Eigen]

    sudo apt-get install libeigen3-dev

[Ipopt]/[Snopt]
Install and set the path to the header and source files through global varialbes in your .bashrc through

    export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.4
    SNOPT_DIR=/home/path/to/snopt/snopt_lib

[ROS]  
Packages: [xpp], catkin, roscpp, tf, kdl_parser, robot_state_publisher, message_runtime, message_generation, std_msgs, geometry_msgs, sensor_msgs, rviz, rosbag, keyboard
      
    sudo apt-get install ros-[ros_distro_name]-[pkg_name]
 

[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm
[rviz]: http://wiki.ros.org/rviz
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[Fa2png]: http://fa2png.io/r/font-awesome/link/