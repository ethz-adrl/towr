/**
 @file    nlp_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Declares the class NlpFacade implementing the Facade Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_

#include "i_visualizer.h"
#include <xpp/utils/state.h>
#include <xpp/utils/eigen_std_conversions.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <memory>
#include "phase.h"

namespace xpp {
namespace opt {

class OptimizationVariables;
class CostContainer;
class ConstraintContainer;
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
  using OptimizationVariablesPtr = std::shared_ptr<OptimizationVariables>;
  using CostContainerPtr         = std::shared_ptr<CostContainer>;
  using ConstraintContainerPtr   = std::shared_ptr<ConstraintContainer>;

  using VisualizerPtr            = std::shared_ptr<IVisualizer>;
  using ComMotionPtrS            = std::shared_ptr<ComMotion>;
  using State                    = xpp::utils::StateLin2d;
  using IpoptPtr                 = Ipopt::SmartPtr<Ipopt::TNLP>;
  using IpoptApplicationPtr      = Ipopt::SmartPtr<Ipopt::IpoptApplication>;
  using VecFoothold              = utils::StdVecEigen2d;

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
                const MotionStructure& motion_structure);

  void AttachNlpObserver(VisualizerPtr& visualizer);
  VecFoothold GetFootholds() const;
  const ComMotionPtrS GetComMotion() const;

private:
  void SolveIpopt(const IpoptPtr& nlp);

  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;

  ComMotionPtrS com_motion_;
  VisualizerPtr visualizer_;

  IpoptApplicationPtr ipopt_app_;
  Ipopt::ApplicationReturnStatus status_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_ */
