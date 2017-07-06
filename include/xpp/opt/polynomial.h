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

namespace xpp {
namespace opt {

/** @brief A polynomial of arbitrary order and dimension.
  */
class Polynomial {
public:

  // x(t)   =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
  // xd(t)  =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
  // xdd(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
  enum PolynomialCoeff { A=0, B, C, D, E, F, G, H, I, J}; // allowed to add more
  using CoeffVec = std::vector<PolynomialCoeff>;

public:
  Polynomial(int order, int dim);
  virtual ~Polynomial() {};

  StateLinXd GetPoint(const double dt) const;
  double GetDerivativeWrtCoeff(MotionDerivative, PolynomialCoeff, double t) const;

  double GetCoefficient(int dim, PolynomialCoeff coeff) const;
  void SetCoefficient(int dim,   PolynomialCoeff coeff, double value);
  void SetCoefficients(PolynomialCoeff coeff, const VectorXd& value);

  CoeffVec GetCoeffIds() const;

private:
  std::vector<VectorXd> coeff_; //!< coefficients values of spline.
  CoeffVec coeff_ids_;          //!< non-zero coefficient indices.
};

} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
