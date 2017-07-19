/**
@file   polynomial.h
@author Alexander Winkler (winklera@ethz.ch)
@date   29.07.2014
@brief  Declares the Polynomial class.
*/
#ifndef _XPP_OPT_UTILS_POLYNOMIAL_H_
#define _XPP_OPT_UTILS_POLYNOMIAL_H_

#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

class Polynomial;
enum PolynomialCoeff { A=0, B, C, D, E, F, G, H, I, J}; // allowed to add more

class PolynomialVars : public Component {
public:
  using PolynomialPtr = std::shared_ptr<Polynomial>;

  PolynomialVars(const std::string& id, const PolynomialPtr& poly);
  virtual ~PolynomialVars() {};

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  StateLinXd GetPoint(double t) const;
  Jacobian GetJacobian(double t, MotionDerivative dxdt) const;

private:
  PolynomialPtr polynomial_;
  int Index(PolynomialCoeff coeff, int dim) const;
};



/** @brief A polynomial of arbitrary order and dimension.
  */
class Polynomial {
public:

  // e.g. 5th-order:
  // x(t)   =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
  // xd(t)  =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
  // xdd(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
  using CoeffIDVec = std::vector<PolynomialCoeff>;

public:
  Polynomial(int order, int dim);
  virtual ~Polynomial() {};

  StateLinXd GetPoint(double t) const;

  void SetConstantPos(const VectorXd& value);
  void SetCoefficient(PolynomialCoeff coeff, int dim, double value);

  // zmp_ attention when using cubic hermite polynomial
  int GetCoeffCount() const { return coeff_ids_.size()*n_dim_; };

  CoeffIDVec GetCoeffIds() const { return coeff_ids_; };

  int GetDimCount() const { return n_dim_; };
  VectorXd GetCoefficients(PolynomialCoeff coeff) const;
  double GetDerivativeWrtCoeff(double t, MotionDerivative, PolynomialCoeff) const;

private:
  std::vector<VectorXd> coeff_;
  CoeffIDVec coeff_ids_;
  int n_dim_;
};





} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
