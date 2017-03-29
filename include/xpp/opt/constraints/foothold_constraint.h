/**
 @file    foothold_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Declares an abstract FootholdConstraint class and one concrete derivation.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_FOOTHOLD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_FOOTHOLD_CONSTRAINT_H_

#include <xpp/opt/endeffectors_motion.h>
#include <xpp/constraint.h>

namespace xpp {
namespace opt {

/** Base class for constraints associated only with the foothold positions
  *
  * This class is responsible for updating the current position of the footholds
  * in order to then calculate costs based on these (move towards goal, avoid
  * obstacles, ...).
  */
class FootholdConstraint : public Constraint {
public:
  using EEMotionPtr   = std::shared_ptr<EndeffectorsMotion>;
  using NominalStance = EEXppPos;

  FootholdConstraint (const EEMotionPtr&,const Vector2d& body_pos,
                      const NominalStance&);
  virtual ~FootholdConstraint ();

private:
  virtual void UpdateConstraintValues () override;
  virtual void UpdateBounds() override;

  EEMotionPtr ee_motion_;
  NominalStance nominal_stance_; // zmp_ express directly in world frame
  Vector2d body_xy_;
  double t_;

  Vector2d GetContactToNominalInWorld(const Vector2d& conctact_W, EndeffectorID leg) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_FOOTHOLD_CONSTRAINT_H_ */
