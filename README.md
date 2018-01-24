
## <img src="https://i.imgur.com/ct8e7T4.png" height="80" />

| ![](https://i.imgur.com/NkL8Haw.gif) | ![](https://i.imgur.com/RrEc2Cd.gif) 
|:-------------------------:|:-------------------------:|
|||



  * xpp_opt: The Trajectory Optimization formulation for floating base systems (ros independent).
  * xpp_opt_ros: A ros wrapper with interactive keyboard input and sending out the optimized motions to be visualized with 

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)** 

This code was developed at the [Agile and Dexterous Robotics Lab](http://www.adrl.ethz.ch/doku.php), ETH Zurich. It is currently maintained by the [Robotics Systems Lab](http://www.rsl.ethz.ch/), ETH Zurich. See the [list of contributors](AUTHORS.txt) for further contributors.

 [<img src="https://i.imgur.com/uCvLs2j.png" height="60" />](http://www.adrl.ethz.ch/doku.php)  &nbsp; &nbsp; &nbsp; &nbsp;    [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html)



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The core algorithm [_towr_](towr) formulates the legged locomotion optimization problem using [ifopt], which can then be solved with any solver. Therefore the dependencies of the core algorithm are:
  * [Eigen] `sudo apt-get install libeigen3-dev`
  * [ifopt] `git clone https://github.com/ethz-adrl/ifopt.git`
  * At least one installed NLP solver, e.g. [Ipopt]. See [ifopt] for details.
  
The wrapper [_towr_ros_](towr_ros) for ROS allows to visualize the produces motions over [rviz] using [xpp]. For this we require the following [ROS] packages:
  * roscpp, rosbag, message_generation, std_msgs, [xpp]: `sudo apt-get install ros-kinetic-[pkg_name]`
  
We also provide an executable for interactive keyboard input to e.g. set desired goal positions. Herefore we use:
  * [ncurses] `sudo apt-get install libncurses5-dev libncursesw5-dev`


 
[Eigen]

    sudo apt-get install libeigen3-dev


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/legged_robotics/xpp.git
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release


## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests

Make sure everything installed correctly by running the unit tests through

    catkin_make run_tests
    
or if you are using [catkin tools].

    catkin build xpp_vis --no-deps --verbose --catkin-make-args run_tests


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage


## <img align="center" height="20" src="https://i.imgur.com/dHQx91Q.png"/> Publications

If you use this work in an academic context, please cite the currently released version <a href="https://doi.org/10.5281/zenodo.1135005"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.1135005.svg" alt="DOI" align="center"></a> as shown [here](https://zenodo.org/record/1135005/export/hx#.Wk3szDCGPmF).


##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/towr/issues).



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
