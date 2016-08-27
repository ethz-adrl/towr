/**
 @file    linear_spline_equations.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/com_spline.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils;

LinearSplineEquations::LinearSplineEquations (const ComSplinePtr& com_spline )
{
  com_spline_ = com_spline;
}

LinearSplineEquations::~LinearSplineEquations ()
{
  // TODO Auto-generated destructor stub
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeInitial (const State2d& init) const
{
  auto derivatives = com_spline_->GetInitialFreeMotions();

  int n_constraints = kDim2d *derivatives.size();
  MatVec M(n_constraints, com_spline_->GetTotalFreeCoeff());

  int i = 0; // constraint count
  double t_global = 0.0;
  for (const Coords3D dim : {X,Y})
  {
    for (auto dxdt :  derivatives) {
      VecScalar diff_dxdt = com_spline_->GetLinearApproxWrtCoeff(t_global, dxdt, dim);
      diff_dxdt.s -= init.GetByIndex(dxdt, dim);
      M.WriteRow(diff_dxdt, i++);
    }
  }

  assert(i==n_constraints);
  return M;
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeFinal (const State2d& final_state) const
{
  auto derivatives = com_spline_->GetFinalFreeMotions();

  int n_constraints = derivatives.size()*kDim2d;
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int c = 0; // constraint count
  double T = com_spline_->GetTotalTime();
  for (const Coords3D dim : {X,Y})
  {
    ComPolynomial last = com_spline_->GetLastPolynomial();
    for (auto dxdt :  derivatives)
    {
      VecScalar diff_dxdt = com_spline_->GetLinearApproxWrtCoeff(T, dxdt, dim);
      diff_dxdt.s -= final_state.GetByIndex(dxdt, dim);
      M.WriteRow(diff_dxdt, c++);
    }
  }

  assert(c==n_constraints);
  return M;
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeJunction () const
{
  auto derivatives = com_spline_->GetJunctionFreeMotions();

  int id_last = com_spline_->GetLastPolynomial().GetId();
  int n_constraints = derivatives.size() * id_last * kDim2d;
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int i = 0; // constraint count
  for (int id = 0; id < id_last; ++id) {
    double T = com_spline_->GetPolynomial(id).GetDuration();
    for (auto dim : {X,Y})
    {
      for (auto dxdt :  derivatives)
      {
        VecScalar curr = com_spline_->GetJacobianWrtCoeff(dxdt, T, id,   dim);
        VecScalar next = com_spline_->GetJacobianWrtCoeff(dxdt, 0, id+1, dim);
        M.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);
  return M;
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeAcceleration (double weight_x, double weight_y) const
{
  std::array<double,2> weight = {weight_x, weight_y}; // weight in x and y direction [1,3]

  // total number of coefficients to be optimized
  int n_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_coeff, n_coeff);

  for (const ComPolynomial& s : com_spline_->GetPolynomials()) {
    std::array<double,8> t_span = utils::cache_exponents<8>(s.GetDuration());

    for (const Coords3D dim : {X,Y}) {
      const int a = com_spline_->Index(s.GetId(), dim, A);
      const int b = com_spline_->Index(s.GetId(), dim, B);
      const int c = com_spline_->Index(s.GetId(), dim, C);
      const int d = com_spline_->Index(s.GetId(), dim, D);

      // for explanation of values see M.Kalakrishnan et al., page 248
      // "Learning, Planning and Control for Quadruped Robots over challenging
      // Terrain", IJRR, 2010
      M.M(a, a) = 400.0 / 7.0      * t_span[7] * weight[dim];
      M.M(a, b) = 40.0             * t_span[6] * weight[dim];
      M.M(a, c) = 120.0 / 5.0      * t_span[5] * weight[dim];
      M.M(a, d) = 10.0             * t_span[4] * weight[dim];
      M.M(b, b) = 144.0 / 5.0      * t_span[5] * weight[dim];
      M.M(b, c) = 18.0             * t_span[4] * weight[dim];
      M.M(b, d) = 8.0              * t_span[3] * weight[dim];
      M.M(c, c) = 12.0             * t_span[3] * weight[dim];
      M.M(c, d) = 6.0              * t_span[2] * weight[dim];
      M.M(d, d) = 4.0              * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < M.M.cols(); ++c)
        for (int r = c + 1; r < M.M.rows(); ++r)
          M.M(r, c) = M.M(c, r);
    }
  }

  return M;
}

} /* namespace zmp */
} /* namespace xpp */

