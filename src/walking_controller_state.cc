/**
 @file    walking_controller_state.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 30, 2016
 @brief   Brief description
 */

#include <xpp/exe/walking_controller_state.h>
#include <xpp/exe/walking_controller.h>

namespace xpp {
namespace exe {

WalkingControllerState::WalkingControllerState ()
{
  // TODO Auto-generated constructor stub
}

WalkingControllerState::~WalkingControllerState ()
{
  // TODO Auto-generated destructor stub
}

WalkingControllerState::StatesMap
WalkingControllerState::BuildStates ()
{
  StatesMap states;

  states.emplace(kFirstPlanning,   std::make_shared<Planning>());
  states.emplace(kExecuting,  std::make_shared<Executing>());
  states.emplace(kRePlanning, std::make_shared<RePlanning>());
  states.emplace(kSleeping,      std::make_shared<Sleep>());

  return states;
}

void Planning::Run(WalkingController* context) const
{
  context->EstimateCurrPose();
  context->PublishCurrentState();
  context->ffsplining_ = false;
  context->BuildPlan();

  context->SetState(kExecuting);
}

void Executing::Run(WalkingController* context) const
{
  bool success = context->ExecuteLoop();
  context->first_time_sending_commands_ = false;
  if (!success)
    context->SetState(kRePlanning);
}

void RePlanning::Run(WalkingController* context) const
{
  context->ffsplining_ = true;
  context->BuildPlan();

  context->SetState(kExecuting);
}

void Sleep::Run(WalkingController* context) const
{
  // do nothing
}

} /* namespace exe */
} /* namespace xpp */
