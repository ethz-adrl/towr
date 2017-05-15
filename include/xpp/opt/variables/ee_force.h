/**
 @file    ee_force.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 15, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_

#include <deque>
#include <vector>

#include <xpp/cartesian_declarations.h>

#include <xpp/bound.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

namespace xpp {
namespace opt {

/** @brief Parameterizes the continuous force over time of a single endeffector.
 */
class EEForce : public Component {
public:
  // remember to adapt index function when changing this
  using PolynomialType = ConstantPolynomial;
//  using PolyXdT        = PolynomialXd<PolynomialType, StateLin1d>;
//  using PolyHelpers    = PolyVecManipulation<PolyXdT>;
//  using VecPolynomials = PolyHelpers::VecPolynomials;

  EEForce (double dt);
  virtual ~EEForce ();

  void AddPhase(double T, bool is_contact);

  virtual VectorXd GetValues() const override;
  virtual VecBound GetBounds() const override;
  virtual void     SetValues(const VectorXd& x) override;

  double GetForce(double t_global) const;
  int Index(double t_global) const;

private:
  double dt_; ///< disretization interval of stance phase [s]
  std::vector<PolynomialType> spline_;
  std::deque<bool> is_in_contact_;
  VecBound bounds_;

  const double max_load_ = 2000.0;
  const double min_load_ = 50.0;

  std::vector<Coords3D> dim_ = {X}; // only z force for now

  void AddContactPhase(double T);
  void AddSwingPhase(double T);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_ */
