TOWR - Trajectory Optimizer for Walking Robots {#mainpage}
---------------------

| *TOWR - A light-weight and extensible C++ library for trajectory optimization for legged robots.* | |
| -------|------ |
| \image html towr_motion_snaps.png | |

This [Eigen]-based library implements variables, costs and constraints that can be 
used to formulate and solve a legged locomotion optimization problem. 
It has been used to generate a variety of motions such as monoped hopping, 
biped walking, or a complete quadruped trotting cycle, while optimizing over 
the gait and step durations in less than 100ms ([paper](https://ieeexplore.ieee.org/document/8283570/)).  

Features:

* Inuitive formulations of variables, cost and constraints using [Eigen].   
* [ifopt] allows to choose between [Ipopt] or [Snopt] to solve the NLP.    
* fast performance due to Eigen sparse matrix exploitation.  
* light-weight ([~6k lines](https://i.imgur.com/gP3gv34.png) of code) makes it easy to use and extend with own formulations.  
* elegant rviz visualization of motion plans using [xpp].  
* [catkin] integration (optional).  


[TOC]

Install {#install}
========================
In case the binaries for you ROS distro don't exist, for more detailed
instructions as well as all dependencies, please checkout the
github 
<a href="https://github.com/ethz-adrl/towr/blob/master/README.md">
README.md
</a>. Shortcut:
\code{.sh}
sudo apt-get install ros-<ros-distro>-towr_ros
\endcode



Run {#run}
=========================
To run the code, type
\code{.sh}
roslaunch towr_ros towr_ros.launch
\endcode


Tune {#tune}
==========================
See [parameters.h](structtowr_1_1Parameters.html#details) for 
an explanation in how to tune the optimization problem.
The number of parameters to tune is relatively small (~10), however, they
do have a large impact on speed and convergence of the optimizer.
 
 
Paper {#paper}
=======================
It is helpful to read up on the theory underlying this code, which is explained 
in this paper 
[DOI: 10.1109/LRA.2018.2798285](https://doi.org/10.1109/LRA.2018.2798285).


Contribute {#contribute}
==========================
We love pull requests. Please see 
<a href="https://github.com/ethz-adrl/towr/blob/master/CONTRIBUTING.md">
CONTRIBUTING.md
</a> if you want to contribute.
See here the list of 
[contributors](https://github.com/ethz-adrl/towr/graphs/contributors) 
who participated in this project.
 

Authors {#authors}
=======================
[Alexander W. Winkler](http://awinkler.me) - Initial Developer & Maintainence


[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[catkin]: http://wiki.ros.org/catkin
[Eigen]: http://eigen.tuxfamily.org

