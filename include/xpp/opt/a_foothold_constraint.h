/**
 @file    a_foothold_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Declares an abstract FootholdConstraint class and one concrete derivation.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"
#include <xpp/utils/eigen_std_conversions.h>
#include <memory>

namespace xpp {
namespace opt {

class ARobotInterface;


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

  utils::StdVecEigen2d footholds_;
  MotionStructure motion_structure_;
};


/** This class constrains the final footholds to be at the nominal stance
  */
class FootholdFinalStanceConstraint : public AFootholdConstraint {
public:
  typedef Eigen::Vector2d Vector2d;
  using RobotPtrU = std::unique_ptr<ARobotInterface>;

  FootholdFinalStanceConstraint(const MotionStructure& motion_structure,
                                const Vector2d& goal_xy, RobotPtrU);
  virtual ~FootholdFinalStanceConstraint();

  virtual VectorXd EvaluateConstraint () const override;
  virtual Jacobian GetJacobianWithRespectTo(std::string var_set) const override;
  virtual VecBound GetBounds() const override;

private:
  Vector2d GetContactToNominalInWorld(const Vector2d& conctact_W, EndeffectorID leg) const;

  Vector2d goal_xy_;
  RobotPtrU robot_;
  std::vector<Contact> final_free_contacts_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_CONSTRAINT_H_ */
