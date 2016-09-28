/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/zmp/com_spline.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils;

ComSpline::ComSpline ()
{
  // TODO Auto-generated constructor stub
}

ComSpline::~ComSpline ()
{
  // TODO Auto-generated destructor stub
}

void
ComSpline::Init (const PhaseVec& phases)
{
  int id = 0;

  for (const auto& phase : phases) {

    if (!phase.IsStep())
    {
//      double t_reaction = 0.15;
//      polynomials_.push_back(ComPolynomial(id++, t_reaction));
//      polynomials_.push_back(ComPolynomial(id++, phase.duration_-t_reaction));
      polynomials_.push_back(ComPolynomial(id++, phase.duration_));
    } else
    {
      int n_splines_per_step = 1;
      for (int i=0; i<n_splines_per_step; ++i) {
        ComPolynomial polynomial(id++, phase.duration_/n_splines_per_step);
        polynomials_.push_back(polynomial);
      }
    }
  }

  splines_initialized_ = true;
}

int
ComSpline::Index (int poly, Coords3D dim, SplineCoeff coeff) const
{
  return NumFreeCoeffPerSpline() * kDim2d * poly + NumFreeCoeffPerSpline() * dim + coeff;
}

int
ComSpline::GetTotalFreeCoeff () const
{
  return polynomials_.size() * NumFreeCoeffPerSpline() * kDim2d;
}

ComSpline::VectorXd
ComSpline::GetCoeffients () const
{
  VectorXd x_abcd(GetTotalFreeCoeff());

  for (const auto& s : polynomials_)
    for (auto dim : { X, Y })
      for (auto coeff :  GetFreeCoeffPerSpline())
        x_abcd[Index(s.GetId(), dim, coeff)] = s.GetCoefficient(dim, coeff);

  return x_abcd;
}

ComSpline::JacobianRow
ComSpline::GetJacobian (double t_global, MotionDerivative posVelAcc,
                        Coords3D dim) const
{
  int id = GetPolynomialID(t_global);
  double t_local = GetLocalTime(t_global);

  return GetJacobianWrtCoeffAtPolynomial(posVelAcc, t_local, id, dim);
}

ComSpline::JacobianRow
ComSpline::GetJacobianWrtCoeffAtPolynomial (MotionDerivative posVelAcc, double t_local, int id,
                                            Coords3D dim) const
{
  assert(0 <= id && id <= polynomials_.back().GetId());

  JacobianRow jac(1, GetTotalFreeCoeff());

  switch (posVelAcc) {
    case kPos: GetJacobianPos (t_local, id, dim, jac); break;
    case kVel: GetJacobianVel (t_local, id, dim, jac); break;
    case kAcc: GetJacobianAcc (t_local, id, dim, jac); break;
    case kJerk:GetJacobianJerk(t_local, id, dim, jac); break;
  }

  return jac;
}

void
ComSpline::CheckIfSplinesInitialized() const
{
  if (!splines_initialized_) {
    throw std::runtime_error("ComSpline.splines_ not initialized. Call Init() first");
  }
}


} /* namespace zmp */
} /* namespace xpp */
