^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package towr_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2019-04-05)
------------------
* Merge pull request (`#56 <https://github.com/ethz-adrl/towr/issues/56>`_) from ethz-adrl/expose-params
* Contributors: Alexander Winkler, awinkler

1.4.0 (2018-07-30)
------------------
* Facilitate towr_ros user extension (`#34 <https://github.com/ethz-adrl/towr/issues/34>`_)
* Add easy plotting of trajectories with rqt_bag
* add option to visualize variable initialization
* Allow shorthand to launch with gdb
* Contributors: Alexander Winkler

1.3.2 (2018-07-17)
------------------
* adapt to more generic ifopt solver interface.
* Contributors: Alexander Winkler

1.3.1 (2018-07-10)
------------------
* Improve API (`#23 <https://github.com/ethz-adrl/towr/issues/23>`_)
* add gait optimization and replay speed to UI
* Add ROS and codefactor badges to readme (`#22 <https://github.com/ethz-adrl/towr/issues/22>`_)
* Contributors: Alexander Winkler

1.3.0 (2018-07-07)
------------------
* visualize goal state on terrain
* Remove redundant rviz terrain visualizer and instead generate
  surface patches directly from height map information
* correctly visualize rviz range-of-motion boxes
* visualize first state
* make GUI static
* Contributors: Alexander Winkler

1.2.2 (2018-07-03)
------------------
* remove controller specifc code from towr_ros
* removed exe subfolder
* moved height map from towr_ros to towr
* moved robots models and gait generator from towr_ros to towr
* move dynamic and kinematic models from towr_ros -> towr
* add ncurses and xterm dependencies to package.xml
* Contributors: Alexander Winkler

1.2.1 (2018-06-30)
------------------
* adapt to revised ifopt version and build structure
* preparation for ANYmal experiments
* Contributors: Alexander Winkler

1.2.0 (2018-06-25)
------------------
* adapt to version 2.0.0 of ifopt (`#17 <https://github.com/ethz-adrl/ifopt/pull/17>`_)
* fix discrepancy between gap height map and rviz visualization
* Contributors: Alexander Winkler

1.1.0 (2018-02-06)
------------------
* improve launch files and class names
* add metapackage towr and move algorithm to towr_core
* create separate ros independent example package "towr_examples"
* adapt to separated ifopt packages
* replaced ros-keyboard dependency with ncurses
* clean-up matlab scripts
* clean and document towr_ros
* add documentaton to angular_state_converter->euler_converter
* moved all robot specific model/gait generators out of towr -> towr_ros
* use only one unified represenatation for nodes and states
* added spline_holder to not always have to reconstruct from variables
* separated spline and node values
* changed to different namespace towr
* renamed pkg from xpp_opt to towr
* Contributors: Alexander Winkler

1.0.0 (2017-09-19)
------------------
