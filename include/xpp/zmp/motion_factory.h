/**
 @file    motion_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares a MotionFactory (Factory Method)
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_

#include <xpp/zmp/phase_info.h>

#include <Eigen/Dense>
#include <memory>

namespace xpp {
namespace zmp {

// at some point this can maybe be replaced with the more general com_motion.h
class ComSpline;
struct SplineTimes;

/** Creates different types of motions based on the input arguments.
  *
  * This methods hides the concrete instantiation from the clients and only
  * exposes a general interface (loose coupling). This class implements the
  * Factory Method.
  */
class MotionFactory {
public:
  typedef std::shared_ptr<ComSpline> ComSplinePtr;
  typedef Eigen::Vector2d Vector2d;

  MotionFactory ();
  virtual ~MotionFactory ();

  /** Creates a spline where all polynomial coefficients are free.
    */
  static ComSplinePtr CreateComMotion(const PhaseVec&);

  /** Creates a spline where the initial position and velocity and the
    * position and velocity at the polynomial junctions are fixed.
    */
  static ComSplinePtr CreateComMotion(const PhaseVec&,
                                      const Vector2d& start_cog_p,
                                      const Vector2d& start_cog_v);


  // motion_ref remove these
  /** Creates a spline where all polynomial coefficients are free.
    */
  static ComSplinePtr CreateComMotion(int step_count,
                                      const SplineTimes& times,
                                      bool insert_initial_stance);

  /** Creates a spline where the initial position and velocity and the
    * position and velocity at the polynomial junctions are fixed.
    */
  static ComSplinePtr CreateComMotion(const Vector2d& start_cog_p,
                                      const Vector2d& start_cog_v,
                                      int step_count,
                                      const SplineTimes& times,
                                      bool insert_initial_stance);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_MOTION_FACTORY_H_ */
