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
  using TimingsVec = std::vector<double>;

  ContactTimings (int ee, const TimingsVec& t);
  virtual ~ContactTimings ();

  // timings for each phase (swing and stance)
  virtual VectorXd GetValues() const override;
  virtual void SetValues(const VectorXd& x) override;
  virtual VecBound GetBounds () const override;

  TimingsVec GetTimings() const { return t_vec_; };

private:

//  void SetTimings(const TimingsVec& t) { t_vec_ = t; };

  TimingsVec t_vec_;
  const double t_max_ = 1; //[s]
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_CONTACT_TIMINGS_H_ */
