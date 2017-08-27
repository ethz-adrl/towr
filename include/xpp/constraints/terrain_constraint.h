/**
 @file    terrain_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_CONSTRAINTS_TERRAIN_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_CONSTRAINTS_TERRAIN_CONSTRAINT_H_

#include <string>

#include <xpp/bound.h>
#include <xpp/composite.h>
#include <xpp/height_map.h>
#include <xpp/variables/phase_nodes.h>

namespace xpp {
namespace opt {


class TerrainConstraint : public Constraint {
public:
  TerrainConstraint (const HeightMap::Ptr& terrain,
                     const OptVarsPtr& opt_vars,
                     std::string ee_motion_id);
  virtual ~TerrainConstraint ();

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;


private:
  EndeffectorNodes::Ptr ee_motion_;
  HeightMap::Ptr terrain_;
};





} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_CONSTRAINTS_TERRAIN_CONSTRAINT_H_ */
