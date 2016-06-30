/*!
 * \file   sl_task.h
 * \author Alexander Winkler
 * \date   Oct 4, 2014
 * \brief  A virtual SL-task that incorporates general functionalities used by
 *         all tasks such as time keeping, logging, failure handling.
 *
 *         Derive from this to create more specific tasks.
 */


#include <xpp/exe/controller.h>
#include <xpp/exe/robot_interface.h>

namespace xpp {
namespace exe {


Controller::Controller()
{
  ResetTime();
}

Controller::~Controller()
{
}

void
Controller::AddRobot (RobotInterfacePtr p_robot)
{
  robot_ = std::move(p_robot); // transfer sole ownership of robot to this class
}

void
Controller::ResetTime()
{
  time_ = 0.0;
}

bool Controller::GetReady()
{
  robot_->StartStateEstimation();

  try {
    ResetTime();
    InitDerivedClassMembers();
  } catch (std::exception& e) {
    std::cerr << "GetReady() caught exception: " << e.what() << "\n\t -> aborting task.\n";
    return false;
  }

  std::cout << "Press any key to continue...";
  std::cin.get(); // use to pause after every iteration

  return true;
}

bool Controller::Run()
{
  try {
    DoSomething(); // state machine->DoSomething()
  } catch (std::exception& e) {
    std::cerr << "Run() caught exception: " << e.what() << "\n\t -> freezing robot.";
    robot_->StopRobot();
  }

  time_ += robot_->GetControlLoopInterval();
  return true;
}

bool Controller::Change()
{
  return true;
}


} // namespace exe
} // namespace xpp

