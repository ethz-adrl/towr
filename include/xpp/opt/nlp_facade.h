/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_


#include <xpp/utils/base_state.h>
#include <xpp/hyq/support_polygon.h>
#include "i_visualizer.h"
#include <xpp/opt/phase_info.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <memory>

namespace xpp {
namespace hyq {
class Foothold;
class SupportPolygonContainer;
}
}

namespace xpp {
namespace opt {

class OptimizationVariables;
class CostContainer;
class ConstraintContainer;
class OptimizationVariablesInterpreter;
class NlpObserver;
class ComPolynomial;
class ComMotion;
class MotionStructure;

/** @brief Simplified interface to setup and solve a Nonlinear-Program.
  *
  * This follows the facade pattern, to hide the complexity of the cost and
  * constraint object creation from the client. The client can optionally
  * pass in a visualizer, but ultimately this class is ROS independent.
  */
class NlpFacade {
public:
  typedef xpp::utils::BaseLin2d State;
  typedef std::shared_ptr<OptimizationVariablesInterpreter> InterpreterPtr;
  typedef Ipopt::SmartPtr<Ipopt::TNLP> IpoptPtr;
  typedef Ipopt::SmartPtr<Ipopt::IpoptApplication> IpoptApplicationPtr;
  typedef std::shared_ptr<NlpObserver> NlpObserverPtr;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;

  typedef std::shared_ptr<OptimizationVariables> OptimizationVariablesPtr;
  typedef std::shared_ptr<CostContainer> CostContainerPtr;
  typedef std::shared_ptr<ConstraintContainer> ConstraintContainerPtr;
  typedef std::shared_ptr<IVisualizer> VisualizerPtr;

  using ComMotionPtrS = std::shared_ptr<ComMotion>;
  using Contacts = xpp::hyq::SupportPolygonContainer;


  NlpFacade (VisualizerPtr visualizer = do_nothing_visualizer);
  virtual ~NlpFacade () {};

  /** @brief Solves the nonlinear program (NLP) of moving the CoG from an initial to a
    * final state while keeping the Zero-Moment-Point (ZMP) inside the support
    * polygons created by the contact points (e.g. feet).
    *
    * This function does not take care of the initialization with previous values,
    * that should be taken care of by new function.
    *
    * @param initial_acc initial acceleration of the CoG
    * @param final_state desired final position, velocity and acceleration of the CoG
    */
  void SolveNlp(const State& initial_state,
                const State& final_state,
                double robot_height,
                const MotionStructure& motion_structure,
                const Contacts& contacts,
                double dt_zmp);

  void AttachVisualizer(VisualizerPtr visualizer);
  NlpObserverPtr GetObserver() const;

  VecFoothold GetFootholds() const;
  ComMotionPtrS GetMotion() const;
  PhaseVec GetPhases() const;

private:
  void SolveIpopt(const IpoptPtr& nlp);

  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;

  NlpObserverPtr nlp_observer_;
  VisualizerPtr visualizer_;

  IpoptApplicationPtr ipopt_app_;
  Ipopt::ApplicationReturnStatus status_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_ */
