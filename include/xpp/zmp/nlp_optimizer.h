/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_


#include <xpp/hyq/foothold.h>
#include <xpp/zmp/spline_container.h>
#include <xpp/zmp/nlp_structure.h>
#include <xpp/hyq/support_polygon_container.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/ros/i_visualizer.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

namespace xpp {
namespace zmp {

class NlpOptimizer {
public:
  typedef xpp::utils::Point2d State;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::ros::IVisualizer IVisualizer;

public:
  NlpOptimizer ();
  virtual
  ~NlpOptimizer () {};


  /**
   * @brief Solves the nonlinear program (NLP) of moving the CoG from an initial to a
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

//  void AttachVisualizer(xpp::ros::IVisualizer&

  VecFoothold GetFootholds() const;
  VecSpline GetSplines();

private:
  OptimizationVariables subject_;

  // these are necessary for correctly interpreting the optimization variables
  std::vector<xpp::hyq::LegID> step_sequence_;
  ContinuousSplineContainer spline_structure_;

  Ipopt::IpoptApplication app_;
  Ipopt::ApplicationReturnStatus status_;

  xpp::ros::IVisualizer& visualizer_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_ */
