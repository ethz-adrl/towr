/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_

#include "endeffectors_motion.h"
#include "endeffector_load.h"
#include "center_of_pressure.h"
#include <xpp/constraint.h>

namespace xpp {
namespace opt {

/** Ensures that the CoP lies within convex hull of contact points p
  *
  * At every discrete node k:
  * g_k = lambda_1*p1 + ... lambda_m*p_m - cop = 0
  */
class SupportAreaConstraint : public Constraint {
public:
  using EEMotionPtr = std::shared_ptr<EndeffectorsMotion>;
  using EELoadPtr   = std::shared_ptr<EndeffectorLoad>;
  using CopPtr      = std::shared_ptr<CenterOfPressure>;

  SupportAreaConstraint ();
  virtual ~SupportAreaConstraint ();

  void Init(const EEMotionPtr&, const EELoadPtr&, const CopPtr&,
            double T, double dt);

  void UpdateConstraintValues () override;
  void UpdateBounds () override;

private:
  EEMotionPtr ee_motion_;
  EELoadPtr ee_load_;
  CopPtr cop_;

  std::vector<double> dts_; ///< discretization of constraint

  void UpdateJacobians() override;
  void UpdateJacobianWithRespectToLoad();
  void UpdateJacobianWithRespectToEEMotion();
  void UpdateJacobianWithRespectToCop();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_ */
