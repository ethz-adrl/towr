<img align="right" src="https://i.imgur.com/qI1Jfyl.gif" width="55%"/>

[<img src="https://i.imgur.com/qliQVx1.png" />](https://ieeexplore.ieee.org/document/8283570 "Go to RA-L paper")

*A light-weight and extensible C++ library for trajectory optimization for legged robots.*

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__towr__ubuntu_bionic_amd64)](http://build.ros.org/view/Mdev/job/Mdev__towr__ubuntu_bionic_amd64/)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](http://docs.ros.org/kinetic/api/towr/html/)
[![ROS hosting](https://img.shields.io/badge/ROS-integration-blue.svg)](http://wiki.ros.org/towr)
![](https://tokei.rs/b1/github/ethz-adrl/towr?category=code)
[![CodeFactor](https://www.codefactor.io/repository/github/ethz-adrl/towr/badge)](https://www.codefactor.io/repository/github/ethz-adrl/towr)
[![License BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://tldrlegal.com/license/bsd-3-clause-license-%28revised%29#fulltext)

A base-set of variables, costs and constraints that can be combined and extended to formulate trajectory optimization problems for legged systems. These implementations have been used to generate a variety of motions such as monoped hopping, biped walking, or a complete quadruped trotting cycle, while optimizing over the gait and step durations in less than 100ms ([paper](https://ieeexplore.ieee.org/document/8283570/)).  

Features:  
:heavy_check_mark: Intuitive and efficient formulation of variables, cost and constraints using [Eigen].   
:heavy_check_mark: [ifopt] enables using the high-performance solvers [Ipopt] and [Snopt].  
:heavy_check_mark: Elegant rviz visualization of motion plans using [xpp].  
:heavy_check_mark: [ROS]/[catkin] integration (optional).  
:heavy_check_mark: Light-weight ([~6k lines](https://i.imgur.com/gP3gv34.png) of code) makes it easy to use and extend.  

<br>

<p align="center">
  <a href="#install">Install</a> •
  <a href="#run">Run</a> •
  <a href="#develop">Develop</a> •
  <a href="#contribute">Contribute</a> •
  <a href="#publications">Publications</a> •
  <a href="#authors">Authors</a>
</p>

[<img src="https://i.imgur.com/8M4v4aP.gif" />](https://youtu.be/0jE46GqzxMM "Show more examples on Youtube")

## Install
The easiest way to install is through the [ROS binaries](http://wiki.ros.org/towr):
```bash
sudo apt-get install ros-<ros-distro>-towr-ros
```

In case these don't yet exist for your distro, there are two ways to build this code from source:
* [Option 1](#towr-with-cmake): core library and hopper-example with pure [CMake].
* [Option 2](#towr-ros-with-catkin) (recommended): core library & GUI & ROS-rviz-visualization built with [catkin] and [ROS]. 


#### <a name="towr-with-cmake"></a> Building with CMake
* Install dependencies [CMake], [Eigen], [Ipopt]:
  ```bash
  sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
  ```
  Install [ifopt], by cloning the repo and then: ``cmake .. && make install`` on your system. 

* Build towr:
  ```bash
  git clone https://github.com/ethz-adrl/towr.git && cd towr/towr
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make
  sudo make install # copies files in this folder to /usr/local/*
  # sudo xargs rm < install_manifest.txt # in case you want to uninstall the above
  ```

* Test ([hopper_example.cc](towr/test/hopper_example.cc)): Generates a motion for a one-legged hopper using Ipopt
  ```bash
  ./towr-example # or ./towr-test if gtest was found
  ```
 
* Use: You can easily customize and add your own constraints and variables to the optimization problem.
  Herefore, add the following to your *CMakeLists.txt*:
  ```cmake
  find_package(towr 1.2 REQUIRED)
  add_executable(main main.cpp) # Your custom variables, costs and constraints added to TOWR
  target_link_libraries(main PUBLIC towr::towr) # adds include directories and libraries
  ```

#### <a name="towr-ros-with-catkin"></a> Building with catkin
We provide a [ROS]-wrapper for the pure cmake towr library, which adds a keyboard interface to modify goal state and motion types as well as visualizes the produces motions plans in rviz using [xpp]. 

* Install dependencies [CMake], [catkin], [Eigen], [Ipopt], [ROS], [xpp], [ncurses], [xterm]:
  ```bash
  sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev libncurses5-dev xterm
  sudo apt-get install ros-<ros-distro>-desktop-full ros-<ros-distro>-xpp
  ```

* Build workspace:
  ```bash
  cd catkin_workspace/src
  git clone https://github.com/ethz-adrl/ifopt.git
  git clone https://github.com/ethz-adrl/towr.git
  cd ..
  catkin_make_isolated -DCMAKE_BUILD_TYPE=Release # or `catkin build`
  source ./devel_isolated/setup.bash
  ```
  
* Use: Include in your catkin project by adding to your *CMakeLists.txt* 
  ```cmake
  add_compile_options(-std=c++11)
  find_package(catkin COMPONENTS towr) 
  include_directories(${catkin_INCLUDE_DIRS})
  target_link_libraries(foo ${catkin_LIBRARIES})
  ```
  Add the following to your *package.xml*:
  ```xml
  <package>
    <depend>towr</depend>
  </package>
  ```
  
## Run
  Launch the program using
  ```bash
  roslaunch towr_ros towr_ros.launch  # debug:=true  (to debug with gdb)
  ```
  Click in the xterm terminal and hit 'o'. 
  
  Information about how to tune the paramters can be found [here](http://docs.ros.org/api/towr/html/group__Parameters.html). 
  
## Develop
#### Library overview
 * The relevant classes and parameters to build on are collected [modules](http://docs.ros.org/api/towr/html/modules.html).
 * A nice graphical overview as UML can be seen [here](http://docs.ros.org/api/towr/html/inherits.html).
 * The [doxygen documentation](http://docs.ros.org/api/towr/html/) provides helpful information for developers.

#### Problem formulation
 * This code formulates the variables, costs and constraints using ifopt, so it makes sense to briefly familiarize with the syntax using [this example].
 * A minimal towr example without ROS, formulating a problem for a one-legged hopper, 
  can be seen [here](towr/test/hopper_example.cc) and is great starting point.
 * We recommend using the ROS infrastructure provided to dynamically visualize, plot and change the problem formulation. To define your own problem using this infrastructure, use this [example](towr_ros/src/towr_ros_app.cc) as a guide. 
 
#### Add your own variables, costs and constraints
 * This library provides a set of variables, costs and constraints to formulate the trajectory optimization problem. An [example formulation](towr/include/towr/nlp_formulation.h) of how to combine these is given, however, this formulation can probably be improved. To add your own e.g. constraint-set, define a class with it's values and derivatives, and then add it to the formulation ```nlp.AddConstraintSet(your_custom_constraints);``` as shown [here](towr/test/hopper_example.cc).

#### Add your own robot
 * Want to add your own robot to towr? Start [here](http://docs.ros.org/en/melodic/api/towr/html/group__Robots.html).
 * To visualize that robot in rviz, see [xpp].

## Contribute
We love pull request, whether its new constraint formulations, additional robot models, bug fixes, unit tests or updating the documentation. Please have a look at [CONTRIBUTING.md](CONTRIBUTING.md) for more information.  
See here the list of [contributors](https://github.com/ethz-adrl/towr/graphs/contributors) who participated in this project.

## Projects using towr
 * https://github.com/popi-mkx3/popi_project

## Publications
All publications underlying this code can be found [here](https://www.alex-winkler.com). 
The core paper is:
 
    @article{winkler18,
      author    = {Winkler, Alexander W and Bellicoso, Dario C and 
                   Hutter, Marco and Buchli, Jonas},
      title     = {Gait and Trajectory Optimization for Legged Systems 
                   through Phase-based End-Effector Parameterization},
      journal   = {IEEE Robotics and Automation Letters (RA-L)},
      year      = {2018},
      month     = {July},
      pages     = {1560-1567},
      volume    = {3},
      doi       = {10.1109/LRA.2018.2798285},
    }
    
A broader overview of the topic of Trajectory optimization and derivation of 
the Single-Rigid-Body Dynamics model used in this work: 
[DOI 10.3929/ethz-b-000272432](https://doi.org/10.3929/ethz-b-000272432)  


## Authors 
[Alexander W. Winkler](https://www.alex-winkler.com) - Initial Work/Maintainer

The work was carried out at the following institutions:

[<img src="https://i.imgur.com/aGOnNTZ.png" height="45" />](https://www.ethz.ch/en.html "ETH Zurich") &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/uCvLs2j.png" height="45" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="45" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")


[A. W. Winkler]: https://awinkler.github.io/publications.html
[CMake]: https://cmake.org/cmake/help/v3.0/
[std_msgs]: http://wiki.ros.org/std_msgs
[roscpp]: http://wiki.ros.org/roscpp
[message_generation]: http://wiki.ros.org/message_generation
[rosbag]: http://wiki.ros.org/rosbag 
[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ANYmal]: http://www.rsl.ethz.ch/robots-media/anymal.html
[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[ifopt_core]: https://github.com/ethz-adrl/ifopt
[ifopt]: https://github.com/ethz-adrl/ifopt
[Ipopt]: https://projects.coin-or.org/Ipopt
[ncurses]: http://invisible-island.net/ncurses/man/ncurses.3x.html
[xterm]: https://linux.die.net/man/1/xterm
[Snopt]: http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm
[rviz]: http://wiki.ros.org/rviz
[catkin]: http://wiki.ros.org/catkin
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[this example]: https://github.com/ethz-adrl/ifopt/blob/master/ifopt_core/test/ifopt/test_vars_constr_cost.h
