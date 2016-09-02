/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace zmp {

ComMotion::ComMotion ()
{
  // TODO Auto-generated constructor stub
}

ComMotion::~ComMotion ()
{
  // TODO Auto-generated destructor stub
}

void
ComMotion::SetCoefficientsZero ()
{
  Eigen::VectorXd coeff(GetTotalFreeCoeff());
  SetCoefficients(coeff.setZero());
}

ComMotion::PhaseInfoVec
ComMotion::GetPhases () const
{
  return phases_;
}

PhaseInfo
ComMotion::GetCurrentPhase (double t_global) const
{
  double t = 0;
  for (const auto& phase: phases_) {
    t += phase.duration_;

    if (t >= t_global) // at junctions, returns previous phase (=)
      return phase;
  }
  assert(false); // this should never be reached
}

ComMotion::VecScalar
ComMotion::GetLinearApproxWrtCoeff (double t_global, MotionDerivative dxdt, Coords3D dim) const
{
  VecScalar linear_approx; // at current coefficient values

  linear_approx.v = GetJacobian(t_global, dxdt, dim);
  linear_approx.s = GetCom(t_global).GetByIndex(dxdt, dim);

  return linear_approx;
}





} /* namespace zmp */
} /* namespace xpp */

