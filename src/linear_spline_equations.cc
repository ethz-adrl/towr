/**
 @file    linear_spline_equations.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_spline_equations.h>

#include <cassert>

#include <xpp/opt/polynomial_spline.h>

namespace xpp {
namespace opt {


LinearSplineEquations::LinearSplineEquations()
{
};

LinearSplineEquations::LinearSplineEquations (const PolynomialSpline& poly_spline)
    :poly_spline_(poly_spline)
{
}

LinearSplineEquations::~LinearSplineEquations ()
{
}

MatVec
LinearSplineEquations::MakeStateConstraint (const StateLinXd& state, double t,
                                            const MotionDerivatives& derivatives) const
{
  int n_dim = poly_spline_.GetNDim();
  int n_constraints = derivatives.size()*n_dim;
  int n_spline_coeff = poly_spline_.GetRows();
  MatVec M(n_constraints, n_spline_coeff);

  int c = 0; // constraint count
  for (int dim=0; dim<n_dim; ++dim)
  {
    for (auto dxdt :  derivatives) {
      VecScalar diff_dxdt;
      diff_dxdt.v = poly_spline_.GetJacobian(t, dxdt, dim);
      diff_dxdt.s = -1 * state.GetByIndex(dxdt)[dim];
      M.WriteRow(diff_dxdt, c++);
    }
  }

  assert(c==n_constraints);
  return M;
}

MatVec
LinearSplineEquations::MakeJunction () const
{
  // acceleration important b/c enforcing system dynamics only once at the
  // junction, so make sure second polynomial also respect that by making
  // its accelerations equal to the first.
  auto derivatives = {kPos, kVel, kAcc};

  auto polynomials = poly_spline_.GetPolynomials();

  int n_dim = poly_spline_.GetNDim();

  int n_junctions = polynomials.size()-1; // because one less junction than poly's.
  int n_constraints = derivatives.size() * n_junctions * n_dim;
  int n_spline_coeff = poly_spline_.GetRows();
  MatVec M(n_constraints, n_spline_coeff);

  int i = 0; // constraint count

  for (int id = 0; id < n_junctions; ++id) {
    double T = polynomials.at(id)->GetDuration();

    for (int dim=0; dim<n_dim; dim++){
      for (auto dxdt :  derivatives) {
        VecScalar curr, next;

        // coefficients are all set to zero
        curr.s = polynomials.at(id)->GetPoint(T).GetByIndex(dxdt)[dim];
        next.s = polynomials.at(id+1)->GetPoint(0.0).GetByIndex(dxdt)[dim];

        curr.v = poly_spline_.GetJacobianWrtCoeffAtPolynomial(dxdt,   T,   id, dim);
        next.v = poly_spline_.GetJacobianWrtCoeffAtPolynomial(dxdt, 0.0, id+1, dim);

        M.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);
  return M;
}

Eigen::MatrixXd
LinearSplineEquations::MakeCostMatrix (const VectorXd& weight, MotionDerivative deriv) const
{
  // total number of coefficients to be optimized
  int n_coeff = poly_spline_.GetRows();
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_coeff, n_coeff);
  int i=0;
  for (const auto& p : poly_spline_.GetPolynomials()) {

    for (int dim=0; dim<poly_spline_.GetNDim(); ++dim){

      double T = p->GetDuration();

      // get only those coefficients that affect this derivative
      auto all_coeff = p->GetCoeffIds();
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
          double deriv_wrt_c1 = p->GetDerivativeWrtCoeff(deriv,c1,T);
          double deriv_wrt_c2 = p->GetDerivativeWrtCoeff(deriv,c2,T);
          double exponent_order = (c1-deriv)+(c2-deriv);
          double val =  (deriv_wrt_c1*deriv_wrt_c2)/(exponent_order+1); //+1 because of integration

          const int idx_row = poly_spline_.Index(i, dim, c1);
          const int idx_col = poly_spline_.Index(i, dim, c2);
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
