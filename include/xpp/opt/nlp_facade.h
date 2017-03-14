/**
 @file    nlp_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Declares the class NlpFacade implementing the Facade Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_

#include "motion_phase.h"
#include <xpp/opt/motion_parameters.h>
#include <xpp/state.h>
#include <memory>

namespace xpp {
namespace opt {

class OptimizationVariables;
class CostContainer;
class ConstraintContainer;
class ComMotion;
class EndeffectorsMotion;
class MotionStructure;
class NLP;

enum NlpSolver { Ipopt, Snopt };

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
  using MotionparamsPtr          = std::shared_ptr<MotionParameters>;
  using ComMotionPtrS            = std::shared_ptr<ComMotion>;
  using EEMotionPtrS             = std::shared_ptr<EndeffectorsMotion>;
  using NLPPtr                   = std::shared_ptr<NLP>;
  using ContactVec               = std::vector<Contact>;

  NlpFacade ();
  virtual ~NlpFacade ();

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
  void BuildNlp(const StateLin2d& initial_state,
                const StateLin2d& final_state,
                EEMotionPtrS& ee_motion,
                const MotionStructure&,
                const MotionparamsPtr&);

  void SolveNlp(NlpSolver solver);

//  ContactVec GetContacts();
  const ComMotionPtrS GetComMotion() const;

private:
  void SolveIpopt();
  void SolveSnopt();
  NLPPtr nlp_;

  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;

  ComMotionPtrS com_motion_;
  EEMotionPtrS ee_motion_;
//  ContactVec contacts_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_ */
