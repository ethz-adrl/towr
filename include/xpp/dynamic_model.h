/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_

#include <vector>

#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

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
  DynamicModel (int ee_count);
  virtual ~DynamicModel ();

  using ComPos    = Vector3d;
  using BaseAcc   = Vector6d;
  using EELoad  = Endeffectors<Vector3d>;
  using EEPos   = EndeffectorsPos;

  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual Jacobian GetJacobianOfAccWrtBaseLin(const Jacobian& jac_base_lin_pos) const = 0;
  virtual Jacobian GetJacobianOfAccWrtBaseAng(const Jacobian& jac_base_ang_pos) const = 0;
  virtual Jacobian GetJacobianofAccWrtForce(const Jacobian& ee_force,
                                            EndeffectorID) const = 0;
  virtual Jacobian GetJacobianofAccWrtEEPos(const Jacobian&,
                                            EndeffectorID) const = 0;

  std::vector<EndeffectorID> GetEEIDs() const;

protected:
  ComPos com_pos_;
  EELoad ee_force_;
  EEPos ee_pos_;

  std::vector<EndeffectorID> ee_ids_;


};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_ */
