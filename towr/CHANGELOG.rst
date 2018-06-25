^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package towr
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2018-06-25)
------------------
* allow building with pure cmake (catkin optional)
* adapt to version 2.0.0 of ifopt `#17 <https://github.com/ethz-adrl/ifopt/pull/17>`_
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
* add observer pattern (spline observes node_values and contact_schedule).
* added spline_holder to not always have to reconstruct from variables
* separated spline and node values
* adapted to changed ifopt namespace (opt -> ifopt)
* removed unused variables in polynomial
* renamed pkg from xpp_opt to towr
* Contributors: Alexander Winkler

1.0.0 (2017-09-19)
------------------
