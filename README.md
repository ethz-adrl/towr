
<img src="https://i.imgur.com/zm2nwF7.png" height="70" />

[<img src="https://i.imgur.com/2Rekk4u.png" />](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf "Open RA-L paper") {:target="_blank"}

**TOWR** - **T**rajectory **O**ptimizer for **W**alking **R**obots,  generates physically feasible motions for legged robots by solving an Optimization Problem. A Centroidal Model of the Dynamics, physical constraints as well as a desired goal position are given to to solver that then produces the motion plan. 

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/ "Go to homepage")**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")       

[<img src="https://i.imgur.com/j8lt5SE.png" />](https://youtu.be/0jE46GqzxMM "Play video on Youtube")


## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The core algorithm [**_towr_**](towr) formulates the legged locomotion optimization problem using [ifopt], which can then be solved with any solver. Therefore the dependencies of the core algorithm are:
  * [Eigen]: Library for linear algebra.
  * [ifopt]: Eigen-based interface to Nonlinear Programming Solver such as Ipopt and Snopt.
  * At least one installed NLP solver, e.g. [Ipopt].
  
The wrapper [**_towr_ros_**](towr_ros) allows to visualize the produced motions over [rviz] using [xpp]. For this we require some [ROS] packages. We also provide an executable for interactive keyboard input to e.g. set desired goal positions, using [ncurses].
  * roscpp, rosbag, message_generation, std_msgs: Standard ROS packages.
  * [xpp]: ROS packages for the visualization of legged robots in rviz.
  * [ncurses]: Preinstalled Linux library for text based GUIs.

Therefore, get [ifopt] and examples running with any NLP solver and install additional dependencies with
      
    sudo apt-get install libeigen3-dev ros-kinetic-[pkg_names] libncurses5-dev libncursesw5-dev


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
