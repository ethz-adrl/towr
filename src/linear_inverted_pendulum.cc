/**
 @file    linear_inverted_pendulum.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_inverted_pendulum.h>

namespace xpp {
namespace opt {

static constexpr double kGravity = 9.80665; // gravity acceleration [m\s^2]

LinearInvertedPendulum::LinearInvertedPendulum ()
{
  // TODO Auto-generated constructor stub
}

LinearInvertedPendulum::~LinearInvertedPendulum ()
{
  // TODO Auto-generated destructor stub
}

void
LinearInvertedPendulum::SetCurrent (const ComPos& pos, const ComVel& vel,
                                    double height)
{
  pos_ = pos;
  vel_ = vel;
  h_ = height;
}

LinearInvertedPendulum::ComAcc
LinearInvertedPendulum::GetDerivative (const Cop& p) const
{
  Eigen::Array2d k1 = (pos_-p)/h_;
  Eigen::Array2d k2 = 2*h_/(h_*h_ + (pos_-p).array().square());
  ComAcc acc = k1*(k2*vel_.array().square() + kGravity);

  double k2_approx = 2./h_;
  ComAcc acc_zmp = k1*kGravity;

  return acc_zmp; // zmp_ this is ignoring velocity
}

} /* namespace opt */
} /* namespace xpp */
