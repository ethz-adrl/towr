/**
 @file    contact_load_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Declares the ContactLoadConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_

#include <memory>
#include <vector>

#include <xpp/endeffectors.h>

#include <xpp/constraint.h>

namespace xpp {
namespace opt {

class EndeffectorLoad;
class ContactSchedule;

/** @brief Makes sure only endeffectors in contact can carry load.
 *
 *  g = lambda_k < contac_flag_k, for all nodes k
 */
class ContactLoadConstraint : public Constraint {
public:
  using EELoadPtr          = std::shared_ptr<EndeffectorLoad>;
  using ContactSchedulePtr = std::shared_ptr<ContactSchedule>;

  ContactLoadConstraint (const OptVarsPtr& opt_vars_container);
  virtual ~ContactLoadConstraint ();

  VectorXd GetConstraintValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  EELoadPtr ee_load_;
  ContactSchedulePtr contact_schedule_;
  std::vector<EndeffectorID> ee_ids_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_ */
