<img align="right" src="https://i.imgur.com/qI1Jfyl.gif" width="55%"/>

[<img src="https://i.imgur.com/qliQVx1.png" />](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf "Open RA-L paper")

*A light-weight and extensible C++ library for trajectory optimization for legged robots.*

<!--- comment until fixing build server
[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/towr/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/towr/job/master/) 
-->
[<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/kinetic/api/towr_core/html/) 

This [Eigen]-based library provides implementations for variables, costs and constraints that can be used to represent a legged locomotion problem. The resulting Nonlinear Programming Problem (NLP) can then be solved with off-the-shelf solvers using the generic optimizer interface [ifopt]. towr has been used to generate a variety of motions such as monoped hopping, biped walking, or a complete quadruped trotting cycle, while optimizing over the gait and step durations in less than 100ms ([paper](https://ieeexplore.ieee.org/document/8283570/)). towr is light-weight ([~6k lines](https://i.imgur.com/gP3gv34.png) of code), which makes it easy to understand, use and extend. It is designed to allow users to add custom constraints, terrains and robots and use already implemented ones when desired. Existing implementations include e.g. constraints that ensure the physical feasiblity of the Single-Rigid-Body Dynamics (SRBD) or constraints to allow arbitrary height maps of the terrain. 

**Author/Maintainer: [:globe_with_meridians: Alexander W. Winkler](https://awinkler.github.io/ "Go to homepage")**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")       

[<img src="https://i.imgur.com/j8lt5SE.png" />](https://youtu.be/0jE46GqzxMM "Play video on Youtube")


## <img align="center" height="15" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies towr
| Name | Min. Ver. | Description |
| --- | --- | --- |
| [CMake] | v3.1.0 | C++ build tool: ```sudo apt-get install cmake```. |
| [Eigen] | v3.2.0 | Library for linear algebra: ```sudo apt-get install libeigen3-dev```. |
| [ifopt] | v2.0.0 | Eigen-based interface to optimization solvers. Only additional dependencies: [Ipopt] and/or [Snopt]. |

Make sure you have these dependencies installed in your system. ifopt can be installed either system wide or, if you're building with [catkin], dropped into your catkin workspace. There are 3 ways to build this package:
* [Build 1](#towr-with-cmake): Cmake-build of core library towr and hopper example
* [Build 2](#towr-with-catkin): Catkin-build of core library towr and hopper example
* [Build 3](#towr-ros-with-catkin): (recommended) Catkin-build of core library & GUI & ros-rviz-visualization (additional dependencies required)


## <a name="towr-with-cmake"></a><img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building towr with CMake
* Install:
  ```bash
  git clone https://github.com/ethz-adrl/towr.git && cd towr/towr
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  sudo xargs rm < install_manifest.txt # in case you want to uninstall the above
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

## <a name="towr-with-catkin"></a><img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building towr with catkin
* Install:
  ```bash
  cd catkin_workspace/src
  git clone https://github.com/ethz-adrl/towr.git
  cd ..
  catkin_make_isolated --pkg towr # `catkin build towr` if you are using catkin command-line tools 
  source ./devel/setup.bash
  ```
  
   
* Test ([hopper_example.cc](towr/test/hopper_example.cc)): Generates a motion for a one-legged hopper using Ipopt
  ```bash
  rosrun towr towr-example # or towr-test if gtest was found
  ```

* Use: Include in your catkin project by adding to your *CMakeLists.txt* 
  ```cmake
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

## <a name="towr-ros-with-catkin"></a><img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building towr_ros and towr with catkin
We also provide a ros-wrapper for towr, which adds a keyboard interface to modify goal state and motion types as well as
visualizes the produces motions plans in rviz using [xpp]. Additional dependencies:

| Name | Min. Ver. | Description |
| --- | --- | --- |
| [ROS] |  indigo | [catkin], [roscpp], [std_msgs], ... ```sudo apt-get install ros-kinetic-desktop-full``` |
| [xpp] | v1.0.6 | Visualization of legged robots in rviz: ```sudo apt-get install ros-kinetic-xpp``` |
| [ncurses] | 5 | Text-based UI: ```sudo apt-get install libncurses5-dev libncursesw5-dev``` |
| [xterm] | 297 | Terminal emulator: ```sudo apt-get install xterm``` |

After having installed all of the above, run
```bash
cd catkin_workspace/src
git clone https://github.com/ethz-adrl/towr.git
cd ..
catkin_make_isolated  # `catkin build` if you are using catkin command-line tools 
source ./devel/setup.bash
```

Launch the program using
```bash
$ roslaunch towr_ros towr_ros.launch
```
Click in the xterm terminal and then hit 'o' for "optimize". Check the box next to HyQ to visualize that URDF.


[<img src="https://i.imgur.com/LNuyEIn.gif" />](https://www.youtube.com/embed/7dP_QTPOyQ8?rel=0&autoplay=1)



## <img align="center" height="15" src="https://i.imgur.com/dHQx91Q.png"/> Publications

Previous versions of this code have been used for a variety of publications. For 
the respective code and the corresponding paper, see [Releases](https://github.com/awinkler/towr/releases).
The theory on the current Release can be cited through this paper:

> A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](https://awinkler.github.io/publications), IEEE Robotics and Automation Letters (RA-L), 2018:

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

##  <img align="center" height="15" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/towr/issues).


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
[Fa2png]: http://fa2png.io/r/font-awesome/link/
