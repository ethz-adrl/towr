/*
 * ellipse.h
 *
 *  Created on: Apr 6, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ELLIPSE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ELLIPSE_H_

namespace xpp
{
namespace utils
{
/**
 * @url http://www.mathwarehouse.com/ellipse/equation-of-ellipse.php
 */
class Ellipse {
public:
  Ellipse () {} ;
  Ellipse (double x_axis_length, double y_axis_length_, double i_rx_m, double i_ry_m);
  virtual
  ~Ellipse () {};




public:
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

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_ELLIPSE_H_ */
