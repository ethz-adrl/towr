/**
@file    phase_info.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 3, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_

namespace xpp {
namespace zmp {

/** Information to represent different types of motion.
  */
class PhaseInfo {
public:
  enum Type {kStancePhase=0, kStepPhase, kFlightPhase} type_;
  int n_completed_steps_;
  int id_;
  double duration_;

  PhaseInfo() : type_(kStancePhase), n_completed_steps_(0),
                id_(-1), duration_(0.0) {};

  /** @param type     Whether this is a stance, step of flight phase.
    * @param n_completed_steps how many steps completed by the previous phases.
    * @param id       Each phase has a unique ID.
    * @param duration How many seconds this phase lasts.
    */
  PhaseInfo(Type type, int n_completed_steps, int id, double duration)
    : type_(type), n_completed_steps_(n_completed_steps), id_(id),
      duration_(duration) {};
};

} /* namespace zmp */
} /* namespace xpp */



#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_ */
