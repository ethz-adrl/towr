/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint classes
 */

#ifndef XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>

#include <xpp/opt/angular_state_converter.h>

#include "composite.h"
#include "time_discretization_constraint.h"

namespace xpp {
namespace opt {

class PolynomialSpline;
class EndeffectorsMotion;

/** @brief Constrains the contact to lie in a box around the nominal stance
  *
  * These constraints are necessary to avoid choosing contact locations
  * that are outside the kinematic reach of the robot. The constraint
  * is defined by Cartesian estimates of the reachability of each endeffector.
  *
  * This constraint calculates the position of of the contact expressed in the
  * current CoM frame and constrains it to lie in a box around the nominal/
  * natural contact position for that leg.
  */
class RangeOfMotionBox : public TimeDiscretizationConstraint {
public:
  using BaseLinear     = std::shared_ptr<PolynomialSpline>;
  using BaseAngular    = std::shared_ptr<PolynomialSpline>;
  using EEMotionPtr    = std::shared_ptr<EndeffectorsMotion>;
  using EESplinePtr    = std::shared_ptr<EndeffectorSpline>;
  using MaxDevXY       = Vector3d;
  using NominalStance  = EndeffectorsPos;

  /**
   * @param dt discretization interval [s] when to check this constraint.
   * @param deviation_xy allowed endeffector deviation from the default (x,y).
   * @param nom nominal endeffector position in base frame.
   */
  RangeOfMotionBox(const OptVarsPtr& opt_vars_container,
                   double dt,
                   const MaxDevXY& deviation_xy,
                   const NominalStance& nom,
                   double T);
  virtual ~RangeOfMotionBox();

private:
  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

  int GetRow(int node, EndeffectorID ee, int dimension) const;

  MaxDevXY max_deviation_from_nominal_;
  NominalStance nominal_stance_;
  BaseLinear base_linear_;
  BaseAngular base_angular_;

  std::vector<EESplinePtr> ee_splines_;
  AngularStateConverter converter_;

  std::vector<Coords3D> dim_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_RANGE_OF_MOTION_CONSTRAINT_H_ */
