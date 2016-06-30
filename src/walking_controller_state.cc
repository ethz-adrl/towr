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

  states.emplace(kFirstPlanning, std::make_shared<FirstPlanning>());
  states.emplace(kExecuting,     std::make_shared<Executing>());
  states.emplace(kUpdateAndExecuting,    std::make_shared<UpdateAndExecuting>());
  states.emplace(kSleeping,      std::make_shared<Sleeping>());

  return states;
}

void FirstPlanning::Run(WalkingController* context) const
{
  context->EstimateCurrPose();
  context->PublishCurrentState();
  context->SetState(kSleeping); // waiting for nlp optimizer to finish
}

void Executing::Run(WalkingController* context) const
{
  bool success = context->ExecuteLoop();

  if (context->TimeExceeded())
    context->SetState(kUpdateAndExecuting);
}

void UpdateAndExecuting::Run(WalkingController* context) const
{
  context->BuildPlan();
  context->SetState(kExecuting);
  context->ExecuteLoop(); // to always send values to the controller
}

void Sleeping::Run(WalkingController* context) const
{
  if (context->Time() > 2.0) {
    context->ResetTime();
    context->SetState(kUpdateAndExecuting);
  }
}

} /* namespace exe */
} /* namespace xpp */
