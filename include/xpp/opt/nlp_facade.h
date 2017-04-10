/**
 @file    nlp_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Declares the class NlpFacade implementing the Facade Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_

#include <xpp/opt/motion_parameters.h>
#include <xpp/robot_state_cartesian.h>
#include <xpp/opt/contact_schedule.h>
#include <xpp/opt/endeffector_load.h> // zmp_ remove from here
#include <xpp/opt/center_of_pressure.h>
#include <memory>

namespace xpp {
namespace opt {

class OptimizationVariablesContainer;
class CostContainer;
class ConstraintContainer;
class BaseMotion;
class EndeffectorsMotion;
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
  using OptimizationVariablesPtr = std::shared_ptr<OptimizationVariablesContainer>;
  using CostContainerPtr         = std::shared_ptr<CostContainer>;
  using ConstraintContainerPtr   = std::shared_ptr<ConstraintContainer>;
  using MotionparamsPtr          = std::shared_ptr<MotionParameters>;
  using ComMotionPtrS            = std::shared_ptr<BaseMotion>;
  using EEMotionPtrS             = std::shared_ptr<EndeffectorsMotion>;
  using ContactSchedulePtr       = std::shared_ptr<ContactSchedule>;
  using NLPPtr                   = std::shared_ptr<NLP>;
  using LoadPtr                  = std::shared_ptr<EndeffectorLoad>;
  using CopPtr                   = std::shared_ptr<CenterOfPressure>;
//  using ContactVec               = std::vector<Contact>;

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
  void OptimizeMotion(const RobotStateCartesian& initial_state,
                      const StateLin2d& final_state,
                      const EEMotionPtrS& ee_motion,
                      const ComMotionPtrS& com_motion,
                      const LoadPtr&,
                      const CopPtr&,
                      const ContactSchedulePtr& contact_schedule,
                      const MotionparamsPtr&,
                      NlpSolver solver);

private:
  void SolveNlp(NlpSolver solver);
  void SolveIpopt();
  void SolveSnopt();
  NLPPtr nlp_;

  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_FACADE_H_ */
