/**
 @file    endeffectors_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_

#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include "contact_schedule.h"
#include "ee_motion.h"

namespace xpp {
namespace opt {

/** Represents the motion of all the endeffectors (feet, hands) of a system.
  *
  * This class is responsible for transforming the scalar parameters into
  * the position, velocity and acceleration of the endeffectors.
  */
class EndeffectorsMotion : public OptimizationVariables {
public:
  using EEState  = Endeffectors<StateLin3d>;
  using VectorXd = Eigen::VectorXd;

  EndeffectorsMotion (const EndeffectorsPos& initial_pos, const ContactSchedule&);
  virtual ~EndeffectorsMotion ();

  VectorXd GetVariables() const override;
  void SetVariables(const VectorXd&) override;
  // order at which the contact position of this endeffector is stored
  JacobianRow GetJacobianWrtOptParams(double t_global, EndeffectorID ee, d2::Coords) const;


  int GetNumberOfEndeffectors() const;
  EEState GetEndeffectors(double t_global) const;
  EEState::Container GetEndeffectorsVec(double t_global) const;
  double GetTotalTime() const;

private:
  int IndexStart(EndeffectorID ee) const;
  Endeffectors<EEMotion> endeffectors_;
  int n_opt_params_ = 0;

  void SetInitialPos(const EndeffectorsPos& initial_pos);
  void SetParameterStructure(const ContactSchedule& contact_schedule);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_ */
