/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef _XPP_ZMP_OPTIMIZER_H_
#define _XPP_ZMP_OPTIMIZER_H_


#include <xpp/hyq/foothold.h>
#include <xpp/zmp/spline_container.h>
#include <xpp/zmp/zmp_spline.h>


/**
@brief Dynamic ("pp" = ++) locomotion ("x"-coordinate)

All functions related to hyq, all utilities and all optimizations are a nested
namespace of this.
 */
namespace xpp {

/**
@brief Zero-Moment-Point (ZMP) optimization

All classes that optimize the CoG trajectory while keeping the ZMP inside the
current support triangle are located in this namespace
 */
namespace zmp {


/**
\class QpOptimizer
\brief Optimizes spline coefficients w.r.t Zero-Moment-Point criteria.

This class provides and interface to optimize the trajectory of the CoM of HyQ
for a specific sequence of footsteps. The resulting trajectory will always keep
the ZMP inside the current support triangle. The QP-solver implemented in
eigen_quadprog.hpp performs the optimization.

 */
class QpFacade {
public:
  typedef std::vector<ZmpSpline> VecSpline;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::Point2d State;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<Foothold> VecFoothold;

public:
  QpFacade() {};
  virtual ~QpFacade() {};

  /**
   * @brief Solves the quadratic program (QP) of moving the CoG from an initial to a
   * final state while keeping the Zero-Moment-Point (ZMP) inside the support
   * polygons created by the contact points (e.g. feet).
   *
   * @param initial_state position, velocity and acceleration of the CoG
   * @param final_state desired final position, velocity and acceleration of the CoG
   * @param start_stance the initial contacts (e.g left front,...) and position
   * @param steps the sequence of footholds (leg and position) to be optimized for
   * @param swing_time time for one step
   * @param stance_time time to transition between disjoint support polygons
   * @param stance_time_initial time before executing the first step
   * @param stance_time_final time after executing the last step to still move the CoG
   * @param robot_height the walking height of the robot (affects ZMP)
   * @return the optimized CoG trajectory
   */
  VecSpline SolveQp(const State& initial_state,
                    const State& final_state,
                    const VecFoothold& start_stance,
                    const VecFoothold& steps,
                    const SplineTimes& times,
                    double robot_height);


private:
  Eigen::VectorXd EigenSolveQuadprog();

  MatVec cost_function_;
  MatVec equality_constraints_;
  MatVec inequality_constraints_;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_OPTIMIZER_H_
