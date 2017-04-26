/**
 @file    linear_spline_equations.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_spline_equations.h>

#include <cassert>

#include <xpp/opt/com_spline.h>
#include <xpp/opt/polynomial_xd.h>

namespace xpp {
namespace opt {

LinearSplineEquations::LinearSplineEquations ()
{
}

LinearSplineEquations::LinearSplineEquations (const ComSpline& com_spline)
{
  com_spline_ = com_spline;
}

LinearSplineEquations::~LinearSplineEquations ()
{
}

MatVec
LinearSplineEquations::MakeStateConstraint (const StateLin3d& state, double t,
                                            const MotionDerivatives& derivatives) const
{
  int n_constraints = derivatives.size()*com_spline_.dim_.size();
  int n_spline_coeff = com_spline_.GetRows();
  MatVec M(n_constraints, n_spline_coeff);

  int c = 0; // constraint count
  for (const Coords3D dim : com_spline_.dim_)
  {
    for (auto dxdt :  derivatives) {
      VecScalar diff_dxdt;
      diff_dxdt.v = com_spline_.GetJacobian(t, dxdt, dim);
      diff_dxdt.s = -1 * state.GetByIndex(dxdt, dim);
      M.WriteRow(diff_dxdt, c++);
    }
  }

  assert(c==n_constraints);
  return M;
}

MatVec
LinearSplineEquations::MakeJunction () const
{
  auto derivatives = {kPos, kVel};

  auto polynomials = com_spline_.GetPolynomials();

  int n_junctions = polynomials.size()-1; // because one less junction than poly's.
  int n_constraints = derivatives.size() * n_junctions * com_spline_.dim_.size();
  int n_spline_coeff = com_spline_.GetRows();
  MatVec M(n_constraints, n_spline_coeff);

  int i = 0; // constraint count

  for (int id = 0; id < n_junctions; ++id) {
    double T = polynomials.at(id).GetDuration();

    for (auto dim : com_spline_.dim_) {
      for (auto dxdt :  derivatives) {
        VecScalar curr, next;

        // coefficients are all set to zero
        curr.s = polynomials.at(id).GetPoint(T).GetByIndex(dxdt, dim);
        next.s = polynomials.at(id+1).GetPoint(0.0).GetByIndex(dxdt, dim);

        curr.v = com_spline_.GetJacobianWrtCoeffAtPolynomial(dxdt,   T,   id, dim);
        next.v = com_spline_.GetJacobianWrtCoeffAtPolynomial(dxdt, 0.0, id+1, dim);

        M.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);
  return M;
}

Eigen::MatrixXd
LinearSplineEquations::MakeCostMatrix (const ValXY& weight, MotionDerivative deriv) const
{
  // total number of coefficients to be optimized
  int n_coeff = com_spline_.GetRows();
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_coeff, n_coeff);
  int i=0;
  for (const auto& p : com_spline_.GetPolynomials()) {

    for (const Coords3D dim : com_spline_.dim_) {

      auto poly = p.GetDim(dim);
      double T = poly.GetDuration();

      // get only those coefficients that affect this derivative
      auto all_coeff = poly.GetCoeffIds();
      Polynomial::CoeffVec coeff_vec;
      for (auto c : all_coeff) {
        if (c >= deriv) coeff_vec.push_back(c);
      }

      for (auto c1 : coeff_vec) {
        for (auto c2 : coeff_vec) {

          // for explanation of values see M.Kalakrishnan et al., page 248
          // "Learning, Planning and Control for Quadruped Robots over challenging
          // Terrain", IJRR, 2010
          // short: "square the values and integrate"
          double deriv_wrt_c1 = poly.GetDerivativeWrtCoeff(deriv,c1,T);
          double deriv_wrt_c2 = poly.GetDerivativeWrtCoeff(deriv,c2,T);
          double exponent_order = (c1-deriv)+(c2-deriv);
          double val =  (deriv_wrt_c1*deriv_wrt_c2)/(exponent_order+1); //+1 because of integration

          const int idx_row = com_spline_.Index(i, dim, c1);
          const int idx_col = com_spline_.Index(i, dim, c2);
          M(idx_row, idx_col) = val*weight[dim];
        }
      }
    }
    i++;
  }

  return M;
}

} /* namespace opt */
} /* namespace xpp */
