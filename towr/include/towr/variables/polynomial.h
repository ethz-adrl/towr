/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_POLYNOMIAL_H_
#define TOWR_VARIABLES_POLYNOMIAL_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "state.h"

namespace towr {

/**
 * @brief A polynomial of arbitrary order and dimension.
 *
 * A polynomial, e.g. 5th order is defined as:
 * f(t)   =   Ft^5 +   Et^4 +  Dt^3 +  Ct^2 + Bt + A
 * fd(t)  =  5Ft^4 +  4Et^3 + 3Dt^2 + 2Ct   + B
 * fdd(t) = 20Ft^3 + 12Et^2 + 6Dt   + 2C
 *
 * This class is responsible for calculating the values f(t) and higher order
 * derivatives from the coefficient values.
 */
class Polynomial {
public:
  enum Coefficients { A=0, B, C, D, E, F, G, H, I, J};
  using CoeffIDVec = std::vector<Coefficients>;
  using VectorXd   = Eigen::VectorXd;

public:
  /**
   * @brief Constructs a polynomial with zero coefficient values.
   * @param poly_order  The highest exponent of t, e.g. 5-th order -> t^5.
   * @param poly_dim    The dimensions of f(t), e.g. x,y,z.
   */
  explicit Polynomial(int poly_order, int poly_dim);
  virtual ~Polynomial() = default;

  /**
   * @returns The state of the polyomial at a specific time t.
   */
  State GetPoint(double t) const;

  /**
   * @brief  The derivative of the polynomial with respect to the coefficients.
   *
   * @param t  The time at which the derivative should be evaluated.
   * @param poly_deriv  Which polynomial derivative f(t), fd(t), function to use.
   * @param coeff  The coefficient with respect to which to calculate the derivative.
   */
  double GetDerivativeWrtCoeff(double t, Dx poly_deriv, Coefficients coeff) const;

protected:
  std::vector<VectorXd> coeff_;

private:
  CoeffIDVec coeff_ids_;
};


/**
 * @brief  Represents a Cubic-Hermite-Polynomial
 *
 * This class is a third-order ("cubic") polynomial, so:
 * f(t)   =  Dt^3 +  Ct^2 + Bt + A
 * fd(t)  = 3Dt^2 + 2Ct   + B
 * fdd(t) = 6Dt   + 2C
 *
 * Instead of setting the polynomial coefficients directly, a third-order
 * polynomial is also fully defined by the value and first-derivative of the
 * start and end of the polynomial as well as the duration. This way of
 * specifying a polynomial is called "Hermite".
 *
 * | Three Cubic-Hermite-Polynomials | |
 * | -------|------ |
 * | \image html nodes.jpg | |
 *
 * See also matlab/cubic_hermite_polynomial.m for generation of derivatives.
 */
class CubicHermitePolynomial : public Polynomial {
public:
  CubicHermitePolynomial(int dim);
  virtual ~CubicHermitePolynomial() = default;


  /**
   * @brief  sets the total duration of the polynomial.
   */
  void SetDuration(double duration);

  /**
   * @brief Fully defines the polynomial by the node values using current duration.
   * @param n0  The value and derivative at the start of the polynomial.
   * @param n1  The value and derivative at the end of the polynomial.
   */
  void SetNodes(const Node& n0, const Node& n1);

  /**
   * @brief updates the coefficients using current nodes and durations.
   */
  void UpdateCoeff();

  /**
   * @brief How the total duration affect the value ("pos") of the polynomial.
   * @param t  The time [0,T] at which the change in value should be observed.
   */
  VectorXd GetDerivativeOfPosWrtDuration(double t) const;

  /**
   * @brief The derivative of the polynomial when changing the start node.
   * @param dxdt  Which polynomial derivative f(t), fd(t) function to use.
   * @param node_deriv  Whether derivative should be w.r.t start-node position or velocity.
   * @param t  The time along the polynomial.
   */
  double GetDerivativeWrtStartNode(Dx dfdt, Dx node_deriv, double t) const;

  /**
   * @brief The derivative of the polynomial when changing the end node.
   * @param dxdt  Which polynomial derivative f(t), fd(t) function to use.
   * @param node_deriv  Whether derivative should be w.r.t end-node position or velocity.
   * @param t  The time along the polynomial.
   */
  double GetDerivativeWrtEndNode(Dx dfdt, Dx node_deriv, double t) const;

  /**
   * @returns the total duration of the polynomial.
   */
  const double GetDuration() const { return T_; };

private:
  double T_;     ///< the total duration of the polynomial.
  Node n0_, n1_; ///< the start and final node comprising the polynomial.

  // see matlab/cubic_hermite_polynomial.m script for derivation
  double GetDerivativeOfPosWrtStartNode(Dx node_deriv, double t_local) const;
  double GetDerivativeOfVelWrtStartNode(Dx node_deriv, double t_local) const;
  double GetDerivativeOfAccWrtStartNode(Dx node_deriv, double t_local) const;

  double GetDerivativeOfPosWrtEndNode(Dx node_deriv, double t_local) const;
  double GetDerivativeOfVelWrtEndNode(Dx node_deriv, double t_local) const;
  double GetDerivativeOfAccWrtEndNode(Dx node_deriv, double t_local) const;
};

} // namespace towr

#endif // TOWR_VARIABLES_POLYNOMIAL_H_
