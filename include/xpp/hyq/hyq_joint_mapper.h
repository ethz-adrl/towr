/**
 @file    hyq_joint_mapper.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 12, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_

#include <xpp/opt/wb_traj_in_out.h> //SplineNode, ArticulatedRobotState

namespace xpp {
namespace hyq {

class HyqState;

class HyqJointMapper {
public:
  static constexpr int kNee = 4;

  using HyqStateVec = std::vector<HyqState>;
  using SplineNode  = xpp::opt::Node<kNee>;
  using ArtiRobVec  = std::vector<xpp::opt::ArticulatedRobotState<kNee>>;

  HyqJointMapper ();
  virtual ~HyqJointMapper ();

  HyqStateVec BuildWholeBodyTrajectoryJoints(const ArtiRobVec&) const;

  SplineNode BuildSplineNode(const HyqState& hyq) const;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_JOINT_MAPPER_H_ */
