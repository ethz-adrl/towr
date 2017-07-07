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


  /** @brief returns a value [0,1] if endeffector is in contact at time t.
   *
   * @param t_global    global time at which the contact state is queried.
   */
  double GetContactValue(double t_global) const;

  JacobianRow GetJacobianOfContactValueWrtTimings(double t_global) const;

private:
  // fix first phase
  enum PhaseType {InContact=0, BreakContact, Flight, MakeContact, PhaseCount};

//  void SetTimings(const TimingsVec& t) { t_vec_ = t; };

  PhaseType GetPhaseType(double t_global) const;
  int Index(double t_global) const;


//  PhaseType Opposite(const PhaseType&) const;

  double eps_ = 0.02; ///< transition time between phases
  TimingsVec GetTVecWithTransitions() const;


  TimingsVec t_vec_;
  const double t_max_ = 1; //[s]
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_CONTACT_TIMINGS_H_ */
