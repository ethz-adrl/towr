/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/i_visualizer.h>
#include <xpp/hyq/support_polygon.h>
#include <xpp/zmp/spline_container.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <memory>

namespace xpp {
namespace hyq {
class Foothold;
class StepSequencePlanner;
}
}

namespace xpp {
namespace zmp {

class OptimizationVariables;
class CostContainer;
class ConstraintContainer;
class OptimizationVariablesInterpreter;
class InterpretingObserver;
class ZmpSpline;

/** @brief Simplified interface to setup and solve a Nonlinear-Program.
  *
  * This follows the facade pattern, to hide the complexity of the cost and
  * constraint object creation from the client. The client can optionally
  * pass in a visualizer, but ultimately this class is ROS independent.
  */
class NlpFacade {
public:
  typedef xpp::utils::Point2d State;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef std::shared_ptr<OptimizationVariablesInterpreter> InterpreterPtr;
  typedef Ipopt::SmartPtr<Ipopt::TNLP> IpoptPtr;
  typedef std::shared_ptr<InterpretingObserver> InterpretingObserverPtr;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef std::vector<ZmpSpline> VecSpline;

  typedef std::shared_ptr<OptimizationVariables> OptimizationVariablesPtr;
  typedef std::shared_ptr<CostContainer> CostContainerPtr;
  typedef std::shared_ptr<ConstraintContainer> ConstraintContainerPtr;
  typedef std::shared_ptr<xpp::hyq::StepSequencePlanner> StepSequencePlannerPtr;

  NlpFacade (IVisualizer& visualizer = do_nothing_visualizer);
  virtual ~NlpFacade () {};


  /** Specifies that initial optimization variables where NLP starts.
    *
    * @param n_spline_coeff number of spline coefficients.
    * @param n_footholds number of footholds.
    */
  void InitializeVariables(int n_spline_coeff, int n_footholds);
  void InitializeVariables(const Eigen::VectorXd& spline_abcd_coeff,
                           const StdVecEigen2d& footholds);


  /** @brief Solves the nonlinear program (NLP) of moving the CoG from an initial to a
    * final state while keeping the Zero-Moment-Point (ZMP) inside the support
    * polygons created by the contact points (e.g. feet).
    *
    * This function does not take care of the initialization with previous values,
    * that should be taken care of by new function.
    *
    * @param initial_acc initial acceleration of the CoG
    * @param final_state desired final position, velocity and acceleration of the CoG
    * @param interpreter holds information about what the opt. variables represent
    */
  void SolveNlp(const State& curr_cog_,
                const State& final_state,
                int curr_swing_leg,
                double robot_height_,
                VecFoothold curr_stance,
                xpp::hyq::MarginValues margins,
                xpp::zmp::SplineTimes spline_times_);

  void AttachVisualizer(IVisualizer& visualizer);
  InterpretingObserverPtr GetObserver() const;

  VecFoothold GetFootholds() const;
  VecSpline GetSplines() const;

private:
  void SolveIpopt(const IpoptPtr& nlp);

  StepSequencePlannerPtr step_sequence_planner_;

  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;

  InterpretingObserverPtr interpreting_observer_;
  IVisualizer* visualizer_;

  Ipopt::IpoptApplication ipopt_solver_;
  Ipopt::ApplicationReturnStatus status_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_ */
