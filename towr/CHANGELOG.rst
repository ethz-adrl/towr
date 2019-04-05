^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package towr
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2019-04-05)
------------------
* Merge pull request (`#56 <https://github.com/ethz-adrl/towr/issues/56>`_) from ethz-adrl/expose-params
* add explanation for assert in phase_durations
* Merge branch 'sweetie-bot-project-feat/access-optimization-parameteres' into expose-params
* Optimize GetPhaseDurations(). Remove unnecessary GetNormalizedPhaseDurations().
* Make GaitGenerator::SetGaits() method public.
* Move default parameter values to header.
* Use pair<double,double> instead of array<double,2> to store bounds.
* Expose contstraints and costs field to user. Remove unnecessary private functions.
* Add missing underscore postfix for bounds_final variables.
* Take into account weights of the costs. (`#51 <https://github.com/ethz-adrl/towr/issues/51>`_)
* Contributors: Alexander Winkler, Mathieu Geisert, awinkler, disRecord

1.4.0 (2018-07-30)
------------------
* Simplify extension / adding own formulation (`#38 <https://github.com/ethz-adrl/towr/issues/38>`_)
* Facilitate towr_ros user extension (`#34 <https://github.com/ethz-adrl/towr/issues/34>`_)
* Greatly simplify phase node formulations (`#33 <https://github.com/ethz-adrl/towr/issues/33>`_) 
* keep overview in github readme, not separated into doxygen & github
* rename CD to SRBD and improve documentation
* rename nodes to node variables
* Contributors: Alexander Winkler

1.3.2 (2018-07-17)
------------------
* adapt to more generic ifopt solver interface.
* Improve doxygen  (`#26 <https://github.com/ethz-adrl/towr/issues/26>`_)
  * add overview on main doxygen landing
  * add doxygen groups to variables/constraints/costs
  * add parameter explanation
* Contributors: Alexander Winkler

1.3.1 (2018-07-10)
------------------
* Improve API (`#23 <https://github.com/ethz-adrl/towr/issues/23>`_)
* Remove redundant total time (duration set by endeffectors)
* Contributors: Alexander Winkler

1.3.0 (2018-07-07)
------------------
* add sample gaits for mono-, bi- and quadruped
* Contributors: Alexander Winkler

1.2.2 (2018-07-03)
------------------
* remove controller specifc code from towr_ros
* moved height map from towr_ros to towr
* moved robots models and gait generator from towr_ros to towr
* move dynamic and kinematic models from towr_ros -> towr
* remove all catkin macros from towr::CMakeLists.txt
* Contributors: Alexander Winkler

1.2.1 (2018-06-30)
------------------
* set parameters for hyq and terrains examples
* rename constraints and variables for more consistency
* renamed main library (towr_core -> towr) and removed ros meta package
* Contributors: Alexander Winkler

1.2.0 (2018-06-25)
------------------
* allow building with pure cmake (catkin optional)
* adapt to version 2.0.0 of ifopt (`#17 <https://github.com/ethz-adrl/ifopt/pull/17>`_)
* add derivative of system dynamics w.r.t angular orientation
* Improve centroidal dynamics model and add continuous base acceleration constraint
* Fix final base and footholds through constraint
* Separate ifopt solver from towr and towr to header+source file
* Contributors: Alexander Winkler

1.1.0 (2018-02-06)
------------------
* add metapackage towr and move algorithm to towr_core
* create separate ros independent example package "towr_ros"
* replaced ros-keyboard dependency with ncurses
* moved all robot specific model/gait generators out of towr -> towr_ros
* use only one unified represenatation for nodes and states
* remove xpp states dependency
* added base_nodes class that derives from node_variables
* add observer pattern (spline observes node_values and contact_schedule)
* added spline_holder to not always have to reconstruct from variables
* separated spline and node values
* adapted to changed ifopt namespace (opt -> ifopt)
* removed unused variables in polynomial
* renamed pkg from xpp_opt to towr
* Contributors: Alexander Winkler

1.0.0 (2017-09-19)
------------------
