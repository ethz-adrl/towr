/**
 @file    a_foothold_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Declares an abstract FootholdConstraint class and one concrete derivation.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_

#include <xpp/a_constraint.h>
#include "motion_structure.h"
#include "eigen_std_conversions.h"
#include "endeffectors_motion.h"
#include <memory>

namespace xpp {
namespace opt {


/** Base class for constraints associated only with the foothold positions
  *
  * This class is responsible for updating the current position of the footholds
  * in order to then calculate costs based on these (move towards goal, avoid
  * obstacles, ...).
  */
class AFootholdConstraint : public AConstraint {
public:
  AFootholdConstraint ();
  virtual ~AFootholdConstraint ();
  void UpdateVariables(const OptimizationVariables*) override;

protected:
  void Init(const MotionStructure&);

  StdVecEigen2d footholds_;
  MotionStructure motion_structure_;
};


/** This class constrains the final footholds to be at the nominal stance
  */
class FootholdFinalStanceConstraint : public AFootholdConstraint {
public:
  using Vector2d       = Eigen::Vector2d;
  using NominalStance  = std::map<EndeffectorID, Eigen::Vector2d>;

  FootholdFinalStanceConstraint(const MotionStructure& motion_structure,
                                const Vector2d& goal_xy, const NominalStance&);
  virtual ~FootholdFinalStanceConstraint();

  virtual VectorXd EvaluateConstraint () const override;
  virtual Jacobian GetJacobianWithRespectTo(std::string var_set) const override;
  virtual VecBound GetBounds() const override;

private:
  Vector2d GetContactToNominalInWorld(const Vector2d& conctact_W, EndeffectorID leg) const;

  Vector2d goal_xy_;
  NominalStance nominal_stance_;
  std::vector<ContactBase> final_free_contacts_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_ */
