/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_


#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/spline_constraints.h>
#include <xpp/ros/zmp_publisher.h>


#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

namespace xpp {
namespace zmp {

class NlpOptimizer {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::Point2d State;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;
  typedef xpp::hyq::Foothold Foothold;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;

public:
  NlpOptimizer ();
  virtual
  ~NlpOptimizer () {};


  void SolveNlp(const State& initial_state,
                const State& final_state,
                const std::vector<xpp::hyq::LegID>& step_sequence,
                const VecFoothold& start_stance,
                VecSpline& opt_splines,
                VecFoothold& final_footholds,
                const Eigen::VectorXd& initial_spline_coeff = Eigen::Vector2d::Zero());



  Ipopt::IpoptApplication app_;
  Ipopt::ApplicationReturnStatus status_;

  xpp::ros::ZmpPublisher zmp_publisher_;

  double swing_time_;
  double stance_time_;
  double stance_time_initial_;
  double stance_time_final_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_ */
