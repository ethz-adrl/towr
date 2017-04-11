/**
 @file    motion_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares a MotionFactory (Factory Method)
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_MOTION_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_MOTION_FACTORY_H_

#include <Eigen/Dense>
#include <memory>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

class BaseMotion;

/** Creates different types of motions based on the input arguments.
  *
  * This methods hides the concrete instantiation from the clients and only
  * exposes a general interface (loose coupling). This class implements the
  * Factory Method.
  */

// zmp_ remove this class
class MotionFactory {
public:
  using ComMotionPtrS = std::shared_ptr<BaseMotion>;
  using PosXY         = Eigen::Vector2d;

  MotionFactory ();
  virtual ~MotionFactory ();

  /** Creates a 5th-order spline where all polynomial coefficients are free.
    */
  static ComMotionPtrS CreateComMotion(double t_total,
                                       int polynomials_per_second,
                                       double height);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_MOTION_FACTORY_H_ */
