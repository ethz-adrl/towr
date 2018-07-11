TOWR - Trajectory Optimizer for Walking Robots {#mainpage}
---------------------

*A light-weight and extensible C++ library for trajectory optimization for legged robots.*





[TOC]

Install {#install}
====================
\code{.sh}
sudo apt-get install ros-<ros-distro>-towr_ros
\endcode
In case the binaries for you distro don't exist, for more detailed
instructions as well as all dependencies, please checkout the
github 
<a href="https://github.com/ethz-adrl/towr/blob/master/README.md">
README.md
</a>.


Run {#run}
==========================
To run the code, type
\code{.sh}
roslaunch towr_ros towr_ros.launch
\endcode


Tune {#tune}
==========================
The number of parameters to tune is relativel small (~10), however, they
do have a large impact on speed and convergence of the optimizer.
 
    
Contribute {#contribute}
==========================
We love pull requests. Please see 
<a href="https://github.com/ethz-adrl/towr/blob/master/CONTRIBUTING.md">
CONTRIBUTING.md
</a>
 
   
Authors {#authors}
=======================
[Alexander W. Winkler](https://github.com/awinkler) 


