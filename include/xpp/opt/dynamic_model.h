/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_

#include <vector>

#include <xpp/endeffectors.h>
#include <xpp/state.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/polynomial_spline.h>

namespace xpp {
namespace opt {

/**
 * @brief Interface to the dynamic system model.
 *
 * This model must implement the relationship between the endeffector forces
 * and the movement of the 6-dimensional base.
 */
class DynamicModel {
public:
  DynamicModel ();
  virtual ~DynamicModel ();

  using ComPos    = Vector3d;
  using ComLinAcc = Vector3d;
  using ComAngAcc = Vector3d;
  using BaseAcc   = Vector6d;

  using EELoad  = Endeffectors<Vector3d>;
  using EEPos   = EndeffectorsPos;
  using BaseLin = PolynomialSpline;
  using BaseAng = PolynomialSpline;

  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const BaseLin&,
                                              double t_global) const = 0;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const BaseAng&,
                                              double t_global) const = 0;
  virtual Jacobian GetJacobianofAccWrtForce(const EndeffectorsForce&,
                                            double t_global,
                                            EndeffectorID) const = 0;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian&,
                                            EndeffectorID) const = 0;

  std::vector<EndeffectorID> GetEEIDs() const;

protected:
  ComPos com_pos_;
  EELoad ee_force_;
  EEPos ee_pos_;


};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_ */
