/**
 @file    dynamic_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_

#include <Eigen/Dense>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffectors_force.h>
#include <xpp/opt/variables/endeffectors_motion.h>

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

  using EELoad = Endeffectors<Vector3d>;
  using EEPos  = EndeffectorsPos;

  void SetCurrent(const ComPos& com, const EELoad&, const EEPos&);

  virtual BaseAcc GetBaseAcceleration() const = 0;

  virtual JacobianRow GetJacobianOfAccWrtBase(const BaseMotion&,
                                              double t,
                                              Coords6D dim) const = 0;

  virtual JacobianRow GetJacobianofAccWrtLoad(const EndeffectorsForce&,
                                              double t,
                                              EndeffectorID,
                                              Coords6D dim) const = 0;

  virtual JacobianRow GetJacobianofAccWrtEEPos(const EndeffectorsMotion&,
                                               double t,
                                               EndeffectorID,
                                               Coords6D dim) const = 0;

  std::vector<EndeffectorID> GetEEIDs() const;

protected:
  ComPos pos_;
  EELoad ee_load_;
  EEPos ee_pos_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_MODEL_H_ */
