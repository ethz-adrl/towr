/**
 @file    endeffectors_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <xpp/opt/constraints/composite.h>

#include "contact_schedule.h"
#include "ee_motion.h"

namespace xpp {
namespace opt {

/** Represents the motion of all the endeffectors (feet, hands) of a system.
  *
  * This class is responsible for transforming the scalar parameters into
  * the position, velocity and acceleration of the endeffectors.
  */
class EndeffectorsMotion : public Composite {
public:
  using EEState  = EndeffectorsState;
  using VectorXd = Eigen::VectorXd;
  using ComponentPtr = std::shared_ptr<EEMotion>;
  using ComponentVec = std::vector<ComponentPtr>;

  EndeffectorsMotion (const EndeffectorsPos& initial_pos, const ContactSchedule&);
  virtual ~EndeffectorsMotion ();

  // order at which the contact position of this endeffector is stored
  int GetNumberOfEndeffectors() const;
  EEState GetEndeffectors(double t_global) const;
  JacobianRow GetJacobianPos(double t_global, EndeffectorID ee, d2::Coords) const;

private:
  int IndexStart(EndeffectorID ee) const;
  ComponentVec endeffectors_; // derived class pointer to access ee specific functions
  std::vector<EndeffectorID> ee_ordered_;

  static ComponentVec BuildEndeffectors(const EndeffectorsPos& initial,
                                        const ContactSchedule&);
  void AddEndffectors(const ComponentVec&);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_ */
