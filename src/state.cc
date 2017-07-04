/*!
 * \file   base_state.h
 * \author Alexander Winkler
 * \date   Jul 4, 2014
 * \brief  Structures to hold the pose (position + orientation) of an object
 */

#include <vector>

#include <xpp/state.h>

namespace xpp {

StateLinXd::StateLinXd ()
{
  // careful: dimension not yet set
}

StateLinXd::StateLinXd (int dim)
{
  kNumDim = dim;
  p_ = VectorXd::Zero(dim);
  v_ = VectorXd::Zero(dim);
  a_ = VectorXd::Zero(dim);
}

StateLinXd::StateLinXd (const VectorXd& _p,
                        const VectorXd& _v,
                        const VectorXd& _a)
    :StateLinXd(_p.rows())
{
  p_ = _p;
  v_ = _v;
  a_ = _a;
}

const VectorXd
StateLinXd::GetByIndex (MotionDerivative deriv) const
{
  switch (deriv) {
    case kPos:  return p_; break;
    case kVel:  return v_; break;
    case kAcc:  return a_; break;
    default: assert(false); // derivative not part of state
  }
}

VectorXd&
StateLinXd::GetByIndex (MotionDerivative deriv)
{
  switch (deriv) {
    case kPos:  return p_; break;
    case kVel:  return v_; break;
    case kAcc:  return a_; break;
    default: assert(false); // derivative not part of state
  }
}

StateLin1d
StateLinXd::GetDimension (int dim) const
{
  assert(dim < kNumDim);

  StateLin1d state1d;
  state1d.p_(0) = p_(dim);
  state1d.v_(0) = v_(dim);
  state1d.a_(0) = a_(dim);
  return state1d;
}

void
StateLinXd::SetDimension (int dim, const StateLin1d& state1d)
{
  assert(dim < kNumDim);

  p_(dim) = state1d.p_(0);
  v_(dim) = state1d.v_(0);
  a_(dim) = state1d.a_(0);
}

StateLin1d::StateLin1d (const StateLinXd& state_xd) : StateLinXd(1)
{
  assert(state_xd.kNumDim == 1);

  p_ = state_xd.p_;
  v_ = state_xd.v_;
  a_ = state_xd.a_;
}

StateLin3d::StateLin3d (const StateLinXd& state_xd) : StateLinXd(3)
{
  assert(state_xd.kNumDim == 3);

  p_ = state_xd.p_;
  v_ = state_xd.v_;
  a_ = state_xd.a_;
}

StateLin2d
StateLin3d::Get2D() const
{
  StateLin2d p2d;
  p2d.p_ = p_.topRows<kDim2d>();
  p2d.v_ = v_.topRows<kDim2d>();
  p2d.a_ = a_.topRows<kDim2d>();
  return p2d;
}

Vector6d
State3d::Get6dVel () const
{
  Vector6d h_xd;
  h_xd.segment(AX, 3) = ang.v;
  h_xd.segment(LX, 3) = lin.v_;
  return h_xd;
}

Vector6d
State3d::Get6dAcc () const
{
  Vector6d h_xdd;
  h_xdd.segment(AX, 3) =  ang.a;
  h_xdd.segment(LX, 3) =  lin.a_;
  return h_xdd;
}

} // namespace xpp

