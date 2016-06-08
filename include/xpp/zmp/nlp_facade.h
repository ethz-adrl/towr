/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/a_observer_visualizer.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/cost_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

#include <xpp/ros/optimization_visualizer.h>

#include <xpp/zmp/interpreting_observer.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

namespace xpp {
namespace zmp {

/** @brief Simplified interface to setup and solve a Nonlinear-Program.
  *
  * This follows the facade pattern, to hide the complexity of the cost and
  * constraint object creation from the client. The client can optionally
  * pass in a visualizer, but ultimately this class is ROS independent.
  */
class NlpFacade {
public:
  typedef xpp::utils::Point2d State;
  typedef OptimizationVariablesInterpreter Interpreter;
  typedef Interpreter::VecFoothold VecFoothold;
  typedef Interpreter::VecSpline VecSpline;
  typedef xpp::ros::IVisualizer IVisualizer;
  typedef Ipopt::SmartPtr<Ipopt::TNLP> IpoptPtr;
  typedef std::shared_ptr<InterpretingObserver> InterpretingObserverPtr;
  typedef xpp::ros::OptimizationVisualizer OptimizationVisualizer;

  NlpFacade (IVisualizer& visualizer = xpp::ros::do_nothing_visualizer);
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
    */
  void SolveNlp(const State& initial_state,
                const State& final_state,
                const std::vector<xpp::hyq::LegID>& step_sequence,
                const VecFoothold& start_stance,
                const SplineTimes& times,
                double robot_height);


  void AttachVisualizer(IVisualizer& visualizer);

  VecFoothold GetFootholds() const;
  VecSpline GetSplines();

private:
  VecFoothold footholds_;
  VecSpline splines_;

  void SolveIpopt(const IpoptPtr& nlp);
  OptimizationVariables opt_variables_;
  ConstraintContainer constraints_;
  CostContainer costs_;

//  Interpreter opt_var_interpreter_;
  InterpretingObserverPtr interpreting_observer_;


  IVisualizer* visualizer_;

  Ipopt::IpoptApplication ipopt_solver_;
  Ipopt::ApplicationReturnStatus status_;


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_FACADE_H_ */
