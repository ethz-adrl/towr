/**
 @file    contact_load_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_

#include "endeffector_load.h"
#include "endeffectors_motion.h"

#include <xpp/a_constraint.h>

namespace xpp {
namespace opt {

class ContactLoadConstraint : public AConstraint {
public:
  ContactLoadConstraint ();
  virtual ~ContactLoadConstraint ();

  void Init(const EndeffectorsMotion&, const EndeffectorLoad& ee_load);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;


private:
  EndeffectorLoad ee_load_;
  EndeffectorsMotion ee_motion_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_ */
