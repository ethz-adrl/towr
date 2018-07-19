TOWR - Trajectory Optimizer for Walking Robots {#mainpage}
=================

*TOWR - A light-weight and extensible C++ library for trajectory optimization for legged robots.* 

\image html towr_motion_snaps.png

This [Eigen]-based library implements variables, costs and constraints that can be used to formulate and solve a legged locomotion optimization problem. It has been used to generate a variety of motions such as monoped hopping, biped walking, or a complete quadruped trotting cycle, while optimizing over the gait and step durations in less than 100ms ([paper](https://ieeexplore.ieee.org/document/8283570/)). 

[TOC]


Install {#install}
===============
 * For installation instructions, see <a href="https://github.com/ethz-adrl/towr/blob/master/README.md">
Github's README.md
</a>. 

Develop {#develop}
==============
 * This code formulates the variables, costs and constraints using [ifopt](https://github.com/ethz-adrl/ifopt), so it makes sense to first briefly familiarize with the syntax using [this example](https://github.com/ethz-adrl/ifopt/blob/master/ifopt_core/test/ifopt/test_vars_constr_cost.h). 

 * To understand the architecture of the code and which are the relevant classes
and parameters to customize or build on, please see [Modules](modules.html). 

Paper {#paper}
=================
 * It is helpful to read up on the theory underlying this code, which is explained 
in this paper 
[DOI: 10.1109/LRA.2018.2798285](https://doi.org/10.1109/LRA.2018.2798285) and
watch this [explanatory video](https://youtu.be/KhWuLvb934g).


Contribute {#contribute}
=================
 * We love pull requests. Please see 
<a href="https://github.com/ethz-adrl/towr/blob/master/CONTRIBUTING.md">
CONTRIBUTING.md
</a> if you want to contribute.
See here the list of 
[contributors](https://github.com/ethz-adrl/towr/graphs/contributors) 
who participated in this project.
 

Authors {#authors}
=================
 * [Alexander W. Winkler](http://awinkler.me) - Initial Developer & Maintainence


[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[catkin]: http://wiki.ros.org/catkin
[Eigen]: http://eigen.tuxfamily.org

