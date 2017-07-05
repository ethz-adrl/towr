/**
 @file    contact_timings.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_CONTACT_TIMINGS_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_CONTACT_TIMINGS_H_

#include "xpp/opt/constraints/composite.h"

namespace xpp {
namespace opt {

class ContactTimings : public Component {
public:
  ContactTimings (int ee, int max_num_steps);
  virtual ~ContactTimings ();

  // timings for each phase (swing and stance)
  virtual VectorXd GetValues() const override { return t_; };
  virtual void SetValues(const VectorXd& x) override { t_= x; };
  virtual VecBound GetBounds () const override;

private:
  VectorXd t_;

  const double t_max_ = 1; //[s]
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_CONTACT_TIMINGS_H_ */
