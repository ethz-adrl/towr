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

/** @brief A polynomial of arbitrary order and dimension.
  */
class Polynomial : public Component {
public:

  // x(t)   =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
  // xd(t)  =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
  // xdd(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
  enum PolynomialCoeff { A=0, B, C, D, E, F, G, H, I, J}; // allowed to add more
  using CoeffVec = std::vector<PolynomialCoeff>;

public:
  Polynomial(int order, int dim, const std::string& id="polynomial");
  virtual ~Polynomial() {};

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  /// jacobian of function below wr.t. values
  StateLinXd GetPoint(double t) const;
  Jacobian GetJacobian(double t, MotionDerivative dxdt) const;

  int GetDimCount() const {return n_dim_; };
  void SetCoefficients(PolynomialCoeff coeff, const VectorXd& value);

private:

  ///< ax,ay,az,bx,by,bz,cx,cy,cz
  VectorXd GetCoefficients(PolynomialCoeff coeff) const;

  int Index(PolynomialCoeff coeff, int dim) const;
  double GetDerivativeWrtCoeff(double t, MotionDerivative, PolynomialCoeff) const;

  CoeffVec coeff_ids_;          //!< non-zero coefficient indices.
  VectorXd all_coeff_;
  int n_dim_;
};





} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
