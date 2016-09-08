/**
 @file    motion_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares a MotionFactory (Factory Method)
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_

#include <xpp/zmp/com_motion.h>

#include <Eigen/Dense>
#include <memory>

namespace xpp {
namespace zmp {

class ComMotion;
class PhaseInfo;

/** Creates different types of motions based on the input arguments.
  *
  * This methods hides the concrete instantiation from the clients and only
  * exposes a general interface (loose coupling). This class implements the
  * Factory Method.
  */
class MotionFactory {
public:
  typedef std::shared_ptr<ComMotion> ComMotionPtrS;
  typedef Eigen::Vector2d Vector2d;
  using PhaseVec = std::vector<PhaseInfo>;

  MotionFactory ();
  virtual ~MotionFactory ();

  /** Creates a spline where all polynomial coefficients are free.
    */
  static ComMotionPtrS CreateComMotion(const PhaseVec&);

  /** Creates a spline where the initial position and velocity and the
    * position and velocity at the polynomial junctions are fixed.
    */
  static ComMotionPtrS CreateComMotion(const PhaseVec&,
                                      const Vector2d& start_cog_p,
                                      const Vector2d& start_cog_v);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_ */
