/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <xpp/utils/endeffectors.h>
#include <map>
#include <memory>

namespace xpp {
namespace opt {

enum MotionTypeID { WalkID, TrottID, CamelID, BoundID };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using MotionTypePtr  = std::shared_ptr<MotionParameters>;
  using EEID           = xpp::utils::EndeffectorID;
  using Swinglegs      = std::vector<EEID>;
  using SwingLegCycle  = std::vector<Swinglegs>;
  using PosXY          = Eigen::Vector2d;
  using NominalStance  = std::map<EEID, PosXY>;
  using ValXY          = std::array<double,2>;

  virtual ~MotionParameters();

  virtual SwingLegCycle GetOneCycle() const = 0;
  virtual NominalStance GetNominalStanceInBase() const = 0;
  ValXY GetMaximumDeviationFromNominal() const;

  MotionTypeID id_;
  double t_phase_;
  double max_step_length_;
  double dt_nodes_; ///< time discretization of trajectory for constraints/costs
  int polynomials_per_phase_;
  bool start_with_stance_;
  ValXY weight_com_motion_xy_;

  double weight_com_motion_cost_;
  double weight_range_of_motion_cost_;
  double weight_polygon_center_cost_;

  static MotionTypePtr MakeMotion(MotionTypeID);

protected:
  ValXY max_dev_xy_;
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
