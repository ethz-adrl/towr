
<img src="https://i.imgur.com/zm2nwF7.png" height="70" />

[<img src="https://i.imgur.com/2Rekk4u.png" />](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf "Open RA-L paper")

**TOWR** - **T**rajectory **O**ptimizer for **W**alking **R**obots,  generates physically feasible motions for legged robots by solving an Optimization Problem. A Centroidal Model of the Dynamics, physical constraints as well as a desired goal position are given to to solver that then produces the motion plan. 

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/ "Go to homepage")**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")       

[<img src="https://i.imgur.com/j8lt5SE.png" />](https://youtu.be/0jE46GqzxMM "Play video on Youtube")


## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Overview

The core algorithm [**_towr_**](towr) formulates the legged locomotion optimization problem using _ifopt_, which can then be solved with any solver. Therefore the dependencies of the core algorithm are:
  * [Eigen]: Library for linear algebra.
  * [ifopt]: Eigen-based interface to Nonlinear Programming Solver such as Ipopt and Snopt.
  * [Ipopt]: Or another supported NLP solver.
  
The wrapper [**_towr_ros_**](towr_ros) allows to visualize the produced motions over _RVIZ_ using _xpp_. For this we require some ROS packages. We also provide an executable for interactive keyboard input to e.g. set desired goal positions, using ncurses.
  * [roscpp], [rosbag], [message_generation], [std_msgs]: Standard ROS packages.
  * [xpp]: ROS packages for the visualization of legged robots in rviz.
  * [ncurses]: Preinstalled Linux library for text based GUIs.


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building
Prior to building this, install [ifopt] and one of the NLP solvers and make sure the example runs (!). Then, install additional libraries, clone this repo into your catkin workspace and compile.

    sudo apt-get libncurses5-dev libncursesw5-dev
    sudo apt-get install ros-kinetic-desktop-full
    sudo apt-get install ros-kinetic-xpp 
    cd catkin_workspace/src
    git clone https://github.com/ethz-adrl/towr.git
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release


## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests
Make sure everything installed correctly by running the unit tests through

    catkin_make run_tests
    
or if you are using [catkin tools].

    catkin build towr --no-deps --verbose --catkin-make-args run_tests


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage
To run a simple example type (not working yet)

    roslaunch towr_ros towr_all.launch



## <img align="center" height="20" src="https://i.imgur.com/dHQx91Q.png"/> Publications
The theory on the current Release is explained in this paper:
* A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf), IEEE Robotics and Automation Letters (RA-L), 2018

Previous version of this code (see [Releases](https://github.com/awinkler/towr/tags)) were used for the following Publications:
* A. W. Winkler, F. Farshidian, D. Pardo, M. Neunert, J. Buchli, [Fast Trajectory Optimization for Legged Robots using Vertex-based ZMP Constraints.](https://awinkler.github.io/publications/mypdfs/17-ral-winkler.pdf), IEEE Robotics and Automation Letters (RA-L), 2017

* A. W. Winkler, F. Farshidian, M. Neunert, D. Pardo, J. Buchli, [Online Walking Motion and Foothold Optimization for Quadruped Locomotion](https://awinkler.github.io/publications/mypdfs/17-icra-winkler.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2017

* A. W. Winkler, C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Planning and Execution of Dynamic Whole-Body Locomotion for a Hydraulic Quadruped on Challenging Terrain](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/winkler15icra.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2015


##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/towr/issues).


[A. W. Winkler]: https://awinkler.github.io/publications.html
[std_msgs]: http://wiki.ros.org/std_msgs
[roscpp]: http://wiki.ros.org/roscpp
[message_generation]: http://wiki.ros.org/message_generation
[rosbag]: http://wiki.ros.org/rosbag 
[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[ifopt]: https://github.com/ethz-adrl/ifopt
[Ipopt]: https://projects.coin-or.org/Ipopt
[ncurses]: http://invisible-island.net/ncurses/man/ncurses.3x.html
[Snopt]: http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm
[rviz]: http://wiki.ros.org/rviz
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[Fa2png]: http://fa2png.io/r/font-awesome/link/
