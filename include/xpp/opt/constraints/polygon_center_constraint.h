/**
 @file    polygon_center_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_

#include <memory>

#include <xpp/constraint.h>

namespace xpp {
namespace opt {

class EndeffectorLoad;
class ContactSchedule;

/** Ensures that vector represented by lambdas lies in center
  *
  * g(lambda) = (lambda_1-1/m)^2 + ... + (lambda_m-1/m)^2 = 0
  *          =>  lambda_1^2 - 2/m*lambda_1 + ... + lambda_m^2 - 2/m*lambda_m  = -1/m
  *
  * where m = number of contacts at each discrete node
  */
class PolygonCenterConstraint : public Constraint {
public:
  using EELoadPtr          = std::shared_ptr<EndeffectorLoad>;
  using ContactSchedulePtr = std::shared_ptr<ContactSchedule>;

  PolygonCenterConstraint (const OptVarsPtr&);
  virtual ~PolygonCenterConstraint ();

  void UpdateConstraintValues () override;
  void UpdateBounds () override;

private:
  EELoadPtr ee_load_;
  ContactSchedulePtr contact_schedule_;

  void UpdateJacobians() override;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_ */
