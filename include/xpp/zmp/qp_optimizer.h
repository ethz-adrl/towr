/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef _XPP_ZMP_OPTIMIZER_H_
#define _XPP_ZMP_OPTIMIZER_H_

#include <xpp/zmp/spline_constraints.h>
#include <xpp/hyq/foothold.h>


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
\class ZmpOptimizer
\brief Optimizes spline coefficients w.r.t Zero-Moment-Point criteria.

This class provides and interface to optimize the trajectory of the CoM of HyQ
for a specific sequence of footsteps. The resulting trajectory will always keep
the ZMP inside the current support triangle. The QP-solver implemented in
eigen_quadprog.hpp performs the optimization.

 */
class QpOptimizer {
public:
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::zmp::SplineConstraints::State State;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::vector<Foothold> VecFoothold;

public:
  QpOptimizer() {};
  virtual ~QpOptimizer() {};

  VecSpline SolveQp(const State& initial_state,
                          const State& final_state,
                          const xpp::hyq::LegDataMap<Foothold>& start_stance,
                          const VecFoothold& steps);


private:
  Eigen::VectorXd EigenSolveQuadprog();

  MatVec cost_function_;
  MatVec equality_constraints_;
  MatVec inequality_constraints_;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_OPTIMIZER_H_
