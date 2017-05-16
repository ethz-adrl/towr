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
#include <xpp/opt/spline.h>

namespace xpp {
namespace opt {

/** @brief Parameterizes the continuous force over time of a single endeffector.
 */
class EEForce : public Component {
public:
  using PolyPtr        = std::shared_ptr<Polynomial>;
  using VecPolynomials = std::vector<PolyPtr>;

  EEForce ();
  virtual ~EEForce ();

  void AddPhase(double T, double dt, bool is_contact);

  virtual VectorXd GetValues() const override;
  virtual VecBound GetBounds() const override;
  virtual void     SetValues(const VectorXd& x) override;

  double GetForce(double t_global) const;
  int Index(double t_global) const;

  // make private again
  Spline spline_;
private:

  VecPolynomials polynomials_;
  std::deque<bool> is_in_contact_;

  const double max_load_ = 2000.0;
  const double min_load_ = 50.0;

  std::vector<Coords3D> dim_ = {X}; // only z force for now

  void AddContactPhase(double T, double dt);
  void AddSwingPhase(double T);

  PolyPtr MakePoly(double T) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_EE_FORCE_H_ */
