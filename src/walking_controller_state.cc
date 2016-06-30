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

  states.emplace(EXECUTING, new Executing());
  states.emplace(PLANNING, new Planning());

  return states;
}

void Planning::Run(WalkingController* context) const
{
  context->PlanTrajectory();
  context->SetState(EXECUTING);
}

void Executing::Run(WalkingController* context) const
{
  bool success = context->ExecuteLoop();
  context->first_time_sending_commands_ = false;
  if (!success)
    context->SetState(PLANNING);
}

} /* namespace exe */
} /* namespace xpp */
