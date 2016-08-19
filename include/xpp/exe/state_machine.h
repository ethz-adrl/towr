/*!
 * \file   state_machine.h
 * \author Alexander Winkler
 * \date   Jan 1, 2016
 */


#ifndef XPP_SL_STATE_MACHINE_H_
#define XPP_SL_STATE_MACHINE_H_

#include "controller.h"

namespace xpp {
namespace exe {

/**
 * Implements the template method pattern by adding a state machine loop to the
 * run function of its parent function.
 *
 * todo think about using the state pattern for this
 */
class StateMachine : public Controller {
public:

protected:
  explicit StateMachine(){};
  virtual ~StateMachine() {};


  /*virtual*/ void GetReadyHook() override {
    state_ = INITIALIZING;
  }


  /*virtual*/ bool RunHook() override
  {
    switch (state_)
    {
      case RUNNING: {
        bool success = ExecuteLoop();
        if (!success)
          state_ = INITIALIZING;
        break;
      }
      case INITIALIZING: {
        PlanTrajectory();
        state_ = RUNNING;
        break;
      }
      case WAITING: {
        break;
      }

    }
    return true;
  }



private:
  enum TaskState{RUNNING, INITIALIZING, WAITING} state_ = INITIALIZING;

  virtual bool ExecuteLoop() = 0;
  virtual void PlanTrajectory() = 0;

};


} // namespace exe
} // namespace xpp

#endif /*XPP_SL_STATE_MACHINE_H_ */
