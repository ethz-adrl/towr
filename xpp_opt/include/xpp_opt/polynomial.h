/**
@file   polynomial.h
@author Alexander Winkler (winklera@ethz.ch)
@date   29.07.2014
@brief  Declares the Polynomial class.
*/
#ifndef _XPP_OPT_UTILS_POLYNOMIAL_H_
#define _XPP_OPT_UTILS_POLYNOMIAL_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <xpp_solve/composite.h>


namespace xpp {


enum PolynomialCoeff { A=0, B, C, D, E, F, G, H, I, J}; // allowed to add more



/** @brief A polynomial of arbitrary order and dimension.
  */
class Polynomial {
public:

  // e.g. 5th-order:
  // x(t)   =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
  // xd(t)  =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
  // xdd(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
  using CoeffIDVec = std::vector<PolynomialCoeff>;
  using Ptr        = std::shared_ptr<Polynomial>;

public:
  Polynomial(int order, int dim);
  virtual ~Polynomial() {};

  StateLinXd GetPoint(double t_local) const;

  void SetCoefficient(PolynomialCoeff coeff, int dim, double value);
  void SetCoefficient(PolynomialCoeff coeff, const VectorXd& value);

  int GetCoeffCount() const { return coeff_ids_.size()*n_dim_; };

  CoeffIDVec GetCoeffIds() const { return coeff_ids_; };

  int GetDimCount() const { return n_dim_; };
  VectorXd GetCoefficients(PolynomialCoeff coeff) const;
  double GetDerivativeWrtCoeff(double t, MotionDerivative, PolynomialCoeff) const;

protected:
  std::vector<VectorXd> coeff_;

private:
  CoeffIDVec coeff_ids_;
  int n_dim_;
};


// see matlab/third_order_poly.m script for derivation
class CubicHermitePoly : public Polynomial {
public:
  enum Side {Start=0, End};
  using Node = std::array<VectorXd,2>; // pos,vel

  CubicHermitePoly(int dim);
  virtual ~CubicHermitePoly();

  void SetNodes(const Node& n0, const Node& n1, double T);

  double GetDerivativeOf(MotionDerivative dxdt, Side, MotionDerivative node_value, double t_local) const;
  double GetDerivativeOfPosWrt(Side, MotionDerivative node_value, double t_local) const;
  double GetDerivativeOfVelWrt(Side, MotionDerivative node_value, double t_local) const;
  double GetDerivativeOfAccWrt(Side, MotionDerivative node_value, double t_local) const;

  /** @brief How the total duration affect the position of the polynomial
   *
   * @param t_local the local polynomial time [0,pT]
   * @param p the percent [0,inf] of the total duration w.r.t the derivative is desired that this T_ represents.
   * @returns the derivative for each dimension (e.g. x,y,z)
   */
  VectorXd GetDerivativeOfPosWrtDuration(double t_local) const;

private:
  double T_;
  Node n0_, n1_;
};


class PolynomialVars : public Component {
public:
  using Ptr = std::shared_ptr<PolynomialVars>;

  PolynomialVars(const std::string& id, const Polynomial::Ptr& poly);
  virtual ~PolynomialVars() {};

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;
  Jacobian GetJacobian(double t_local, MotionDerivative dxdt) const;

  Polynomial::Ptr GetPolynomial() const { return polynomial_; };

private:
  Polynomial::Ptr polynomial_;
  int Index(PolynomialCoeff coeff, int dim) const;
};

} // namespace xpp

#endif // _XPP_OPT_UTILS_POLYNOMIAL_H_
