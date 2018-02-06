<img src="https://i.imgur.com/zm2nwF7.png" height="70" />

[<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/api/towr_core/html/index.html)

[<img src="https://i.imgur.com/2Rekk4u.png" />](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf "Open RA-L paper")
**TOWR** - **T**rajectory **O**ptimizer for **W**alking **R**obots, generates physically feasible motions for legged robots by solving an optimization problem. A Centroidal model of the dynamics, physical constraints as well as a desired goal position are given to the solver that then generates the motion plan. _TOWR_ generates 5 step monoped hopping, biped walking, or a complete quadruped trotting cycle, while optimizing over the gait and step durations, in less than **100ms**.

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/ "Go to homepage")**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")       

[<img src="https://i.imgur.com/j8lt5SE.png" />](https://youtu.be/0jE46GqzxMM "Play video on Youtube")


## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Overview

* [**_towr_core_**](towr_core) [(API)](http://docs.ros.org/api/towr_core/html/index.html): The core algorithm formulates the legged locomotion optimization problem using _ifopt_, which can then be solved with any solver. Therefore the dependencies of the core algorithm are:
    * [Eigen]: Library for linear algebra.
    * [ifopt_core]: Eigen-based interface to Nonlinear Programming solvers such as Ipopt and Snopt.
  
* [**_towr_examples_**](towr_examples): Provides an example executable solving [towr_core](towr_core) with [Ipopt] to generate a one-legged hopper motion-plan. Additional dependencies:
    * [Ipopt]: 3rd party NLP solver, [Snopt] can also be used.
  
* [**_towr_ros_**](towr_ros) [(API)](http://docs.ros.org/api/towr_ros/html/index.html): Formulates a variety of robots (Monoped, biped, [HyQ], [ANYmal]) and terrains and a keyboard user interface to switch between them. It also allows to visualize the produced motions in _rviz_ using _xpp_. Additional dependencies:
    * [roscpp], [rosbag], [message_generation], [std_msgs]: Standard ROS packages.
    * [xpp]: ROS packages for the visualization of legged robots in rviz.
    * [ncurses], [xterm]: Preinstalled on most Linux distributions.


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building
Prior to building this, install [ifopt] and one of the NLP solvers and make sure the example runs (!). Then, install additional libraries, clone this repo into your catkin workspace and compile.

    sudo apt-get libncurses5-dev libncursesw5-dev xterm
    sudo apt-get install ros-kinetic-desktop-full
    sudo apt-get install ros-kinetic-xpp
    
    cd catkin_workspace/src
    git clone https://github.com/ethz-adrl/towr.git
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release


## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests
Make sure everything installed correctly by running the unit tests through

    catkin_make run_tests


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage
To run the simplest [example](towr_examples/example.cc)
     
    rosrun towr_examples towr_ipopt_example

For a more advanced [example](towr_ros/src/towr_ros.cc) with interactive keyboard input and ROS visualization,
launch the file below and then hit 'o' for "optimize"

    roslaunch towr_ros towr_ros.launch



## <img align="center" height="20" src="https://i.imgur.com/dHQx91Q.png"/> Publications
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
      month     = {may},
      pages     = {},
      doi       = {},
    }

##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/towr/issues).


[A. W. Winkler]: https://awinkler.github.io/publications.html
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
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[Fa2png]: http://fa2png.io/r/font-awesome/link/
