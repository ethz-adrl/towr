/**
 @file    linear_inverted_pendulum.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include "dynamic_model.h"
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

/**
 * @brief A Linear Inverted Pendulum to model the dynamics of the system.
 */
class LIPModel : public DynamicModel {
public:
  using Cop = Vector2d;

  LIPModel();
  virtual ~LIPModel();

  virtual BaseAcc GetBaseAcceleration() const override;


  virtual Jacobian GetJacobianOfAccWrtBaseAng(const BaseLin&, double t_global) const override;
  virtual Jacobian GetJacobianOfAccWrtBaseLin(const BaseAng&, double t_global) const override;
  virtual Jacobian GetJacobianofAccWrtForce(const EndeffectorsForce&,
                                   double t_global,
                                   EndeffectorID) const override;
  virtual Jacobian GetJacobianofAccWrtEEPos(const EndeffectorsMotion&, double t_global,
                                    EndeffectorID) const override;

private:
  const double h_ = 0.58;
  const double m_ = 80; /// mass of robot

  ComLinAcc GetLinearAcceleration() const;
  double GetDerivativeOfAccWrtLoad(EndeffectorID, Coords3D dim) const;
  double GetDerivativeOfAccWrtEEPos(EndeffectorID, Coords3D dim) const; // same for x and y direction

  Cop CalculateCop() const;
  Cop GetDerivativeOfCopWrtLoad(EndeffectorID) const;
//  double GetDerivativeOfCopWrtEEPos(EndeffectorID) const;
  double GetLoadSum() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_ */
