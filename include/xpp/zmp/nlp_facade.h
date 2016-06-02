/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_


#include <xpp/hyq/foothold.h>
#include <xpp/zmp/zmp_spline.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/a_observer_visualizer.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

// this looks like i need the factory method
#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/constraint_container.h>
// cost function stuff
#include <xpp/zmp/a_quadratic_cost.h>
#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/total_acceleration_equation.h>
#include <xpp/zmp/cost_container.h>


namespace xpp {
namespace zmp {

class NlpFacade {
public:
  typedef xpp::utils::Point2d State;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef std::vector<ZmpSpline> VecSpline;
  typedef xpp::ros::IVisualizer IVisualizer;
  typedef Ipopt::SmartPtr<Ipopt::TNLP> IpoptPtr;

  NlpFacade (AObserverVisualizer& visualizer = do_nothing_observer_visualizer);
  virtual ~NlpFacade () {};

  /** @brief Solves the nonlinear program (NLP) of moving the CoG from an initial to a
    * final state while keeping the Zero-Moment-Point (ZMP) inside the support
    * polygons created by the contact points (e.g. feet).
    *
    * @param initial_state position, velocity and acceleration of the CoG
    * @param final_state desired final position, velocity and acceleration of the CoG
    * @param step_sequence the sequence (e.g. left hind, left front,...) of steps to be taken
    * @param start_stance the initial contacts (e.g left front,...) and position
    * @param times the duration of the different splines
    * @param robot_height the walking height of the robot (affects ZMP)
    * @param[out] opt_splines the optimized CoG trajectory
    * @param[out] opt_footholds the optimized foothold positions
    */
  void SolveNlp(const State& initial_state,
                const State& final_state,
                const std::vector<xpp::hyq::LegID>& step_sequence,
                const VecFoothold& start_stance,
                const SplineTimes& times,
                double robot_height);

  void AttachVisualizer(AObserverVisualizer& visualizer);

  VecFoothold GetFootholds() const;
  VecSpline GetSplines();

private:
  void SolveIpopt(const IpoptPtr& nlp);
  OptimizationVariables subject_;

  ConstraintContainer constraints_;
  LinearEqualityConstraint c_acc_, c_final_, c_junction_;
  ZmpConstraint c_zmp_;
  RangeOfMotionConstraint c_rom_;

  CostContainer cost_container_;
  AQuadraticCost cost_acc_;
  RangeOfMotionCost cost_rom_;


  Ipopt::IpoptApplication app_;
  Ipopt::ApplicationReturnStatus status_;

  AObserverVisualizer* visualizer_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_ */
