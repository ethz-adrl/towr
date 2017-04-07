/**
 @file    support_area_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_

#include <xpp/opt/endeffectors_motion.h>
#include <xpp/opt/endeffector_load.h>
#include <xpp/opt/center_of_pressure.h>
#include <xpp/time_discretization_constraint.h>

namespace xpp {
namespace opt {

/** Ensures that the CoP lies within convex hull of contact points p
  *
  * At every discrete node k:
  * g_k = lambda_1*p1 + ... lambda_m*p_m - cop = 0
  */
class SupportAreaConstraint : public TimeDiscretizationConstraint {
public:
  using EEMotionPtr = std::shared_ptr<EndeffectorsMotion>;
  using EELoadPtr   = std::shared_ptr<EndeffectorLoad>;
  using CopPtr      = std::shared_ptr<CenterOfPressure>;

  SupportAreaConstraint (const EEMotionPtr&, const EELoadPtr&, const CopPtr&,
                         double dt);
  virtual ~SupportAreaConstraint ();

private:
  EEMotionPtr ee_motion_;
  EELoadPtr ee_load_;
  CopPtr cop_;

  void UpdateConstraintAtInstance (double t, int k) override;
  void UpdateBoundsAtInstance (double t, int k) override;
  void UpdateJacobianAtInstance(double t, int k) override;
  int GetConstraintNr(int node, int dimension) const;

  void UpdateJacobianWithRespectToLoad(double t, int k);
  void UpdateJacobianWithRespectToEEMotion(double t, int k);
  void UpdateJacobianWithRespectToCop(double t, int k);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_ */
