/**
 @file    ellipse.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ELLIPSE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ELLIPSE_H_

namespace xpp {
namespace zmp {

/** Defines an ellipse with distances to its center and edges
  *
  * @url http://www.mathwarehouse.com/ellipse/equation-of-ellipse.php
  */
class Ellipse {
public:
  Ellipse ();
  Ellipse (double x_axis_length, double y_axis_length_, double i_rx_m, double i_ry_m);
  virtual ~Ellipse () {};

  double DistanceToEdge(double i_rx_p, double i_ry_p) const;

private:
  double x_axis_length_;
  double y_axis_length_;
  // distance to midpoint m of ellipse
  double i_rx_m_;
  double i_ry_m_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ELLIPSE_H_ */
