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
  int n_constraints = kDim2d *1; // acceleration in x and y direction
  MatVec M(n_constraints, com_spline_->GetTotalFreeCoeff());

  int i = 0; // constraint count
  double t = 0.0;
  int spline_id = 0;
  for (const Coords3D dim : {X,Y})
  {
    VecScalar acc = com_spline_->ExpressComThroughCoeff(kAcc, t, spline_id, dim);
    acc.s -= -init.a(dim);
    M.WriteRow(acc, i++);
  }

  assert(i==n_constraints);
  return M;
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeFinal (const State2d& final_state) const
{
  int n_constraints = 3*kDim2d; // pos, vel, acc
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int c = 0; // constraint count
  for (const Coords3D dim : {X,Y})
  {
    ComPolynomial last = com_spline_->GetLastPolynomial();
    int K = last.GetId();
    double T = last.GetDuration();

    VecScalar pos = com_spline_->ExpressComThroughCoeff(kPos, T, K, dim);
    pos.s -= final_state.p(dim);
    M.WriteRow(pos, c++);

    VecScalar vel = com_spline_->ExpressComThroughCoeff(kVel, T, K, dim);
    vel.s -= final_state.v(dim);
    M.WriteRow(vel, c++);

    VecScalar acc = com_spline_->ExpressComThroughCoeff(kAcc, T, K, dim);
    acc.s -= final_state.a(dim);
    M.WriteRow(acc, c++);
  }

  assert(c==n_constraints);
  return M;
}

LinearSplineEquations::MatVec
LinearSplineEquations::MakeJunction () const
{
  // jerk ensures that minimum acceleration cost function cannot trick
  // by moving all the big jumps in acc to the junction to save cost.
  std::vector<PosVelAcc> derivative = {kAcc, kJerk};

  int id_last = com_spline_->GetLastPolynomial().GetId();
  int n_constraints = derivative.size() * id_last * kDim2d;
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int i = 0; // constraint count
  for (int id = 0; id < id_last; ++id) {
    double T = com_spline_->GetPolynomial(id).GetDuration();
    for (auto dim : {X,Y}) {
      for (auto pva :  derivative) {
        VecScalar curr = com_spline_->ExpressComThroughCoeff(pva, T, id,   dim);
        VecScalar next = com_spline_->ExpressComThroughCoeff(pva, 0, id+1, dim);
        M.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);
  return M;
}

} /* namespace zmp */
} /* namespace xpp */
