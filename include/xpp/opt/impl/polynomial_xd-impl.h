/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <xpp/opt/polynomial_xd.h>

namespace xpp {
namespace opt {

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::PolynomialXd (int id, double duration)
{
  SetId(id);
  SetBoundary(duration, Point(), Point());
}

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::~PolynomialXd ()
{
}

template<typename PolynomialType, typename PointType>
double
PolynomialXd<PolynomialType, PointType>::GetDuration () const
{
  // all polynomials have same duration, so just return duration of X
  return polynomials_.at(X).GetDuration();
}

template<typename PolynomialType, typename PointType>
typename PolynomialXd<PolynomialType, PointType>::Vector
PolynomialXd<PolynomialType, PointType>::GetState (MotionDerivative pos_vel_acc_jerk,
                                                          double t) const
{
  Point p;
  GetPoint(t, p);
  return p.GetByIndex(pos_vel_acc_jerk);
}

template<typename PolynomialType, typename PointType>
double
PolynomialXd<PolynomialType, PointType>::GetCoefficient (int dim, PolyCoeff coeff) const
{
  return polynomials_.at(dim).GetCoefficient(coeff);
}

template<typename PolynomialType, typename PointType>
void
PolynomialXd<PolynomialType, PointType>::SetCoefficients (int dim, PolyCoeff coeff,
                                                          double value)
{
  polynomials_.at(dim).SetCoefficient(coeff,value);
}

template<typename PolynomialType, typename PointType>
void PolynomialXd<PolynomialType, PointType>::SetBoundary(double T,
                                                          const Point& start,
                                                          const Point& end)
{
  for (int dim=X; dim<kNumDim; ++dim)
    polynomials_.at(dim).SetBoundary(T, start.Get1d(dim), end.Get1d(dim));
}

template<typename PolynomialType, typename PointType>
bool PolynomialXd<PolynomialType, PointType>::GetPoint(const double dt,
                                                              Point& p) const
{
  StateLin1d point1d;

  for (int dim=X; dim<kNumDim; ++dim) {
    polynomials_.at(dim).GetPoint(dt, point1d);
    p.SetDimension(point1d, dim);
  }

  return true;
}


} // namespace opt
} // namespace xpp
