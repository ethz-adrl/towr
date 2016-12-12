/**
 @file    hyq_joint_mapper.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 12, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_

#include <xpp/hyq/hyq_state.h>
#include <xpp/hyq/hyq_spliner.h> //zmp_, possibly remove dependecy on hyq_spliner

namespace xpp {
namespace hyq {

class HyqJointMapper {
public:
  using HyqStateVec   = std::vector<HyqState>;
  using ArtiRobVec    = xpp::hyq::HyqSpliner::ArtiRobVec;

  HyqJointMapper ();
  virtual ~HyqJointMapper ();

  HyqStateVec BuildWholeBodyTrajectoryJoints(const ArtiRobVec&) const;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_ */
