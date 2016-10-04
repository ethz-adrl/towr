/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <xpp/utils/polynomial_xd.h>

namespace xpp {
namespace utils {

template<typename PolynomialType, size_t N_DIM, typename PointType>
void
PolynomialXd<PolynomialType, N_DIM, PointType>::SetDuration (double _duration)
{
  for (int dim=X; dim<kNumDim; ++dim)
    polynomials_.at(dim).duration = _duration;
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
double
PolynomialXd<PolynomialType, N_DIM, PointType>::GetDuration () const
{
  // all polynomials have same duration, so just return duration of X
  return polynomials_.at(X).duration;
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
typename PolynomialXd<PolynomialType, N_DIM, PointType>::Vector
PolynomialXd<PolynomialType, N_DIM, PointType>::GetState (MotionDerivative pos_vel_acc_jerk,
                                                          double t) const
{
  Point p;
  GetPoint(t, p);
  return p.GetByIndex(pos_vel_acc_jerk);
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
double
PolynomialXd<PolynomialType, N_DIM, PointType>::GetCoefficient (int dim, SplineCoeff coeff) const
{
  return polynomials_.at(dim).c[coeff];
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
void
PolynomialXd<PolynomialType, N_DIM, PointType>::SetCoefficients (int dim,
                                                                 SplineCoeff coeff,
                                                                 double value)
{
  polynomials_.at(dim).c[coeff] = value;
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
void PolynomialXd<PolynomialType, N_DIM, PointType>::SetBoundary(double T,
                                                                 const Point& start,
                                                                 const Point& end)
{
  Polynomial::Point1d start1d;
  Polynomial::Point1d end1d;

  for (int dim=X; dim<kNumDim; ++dim) {
    // convert to 1D points
    start1d.x   = start.p(dim);  end1d.x   = end.p(dim);
    start1d.xd  = start.v(dim);  end1d.xd  = end.v(dim);
    start1d.xdd = start.a(dim);  end1d.xdd = end.a(dim);

    polynomials_.at(dim).SetBoundary(T, start1d, end1d);
  }
}

template<typename PolynomialType, size_t N_DIM, typename PointType>
bool PolynomialXd<PolynomialType, N_DIM, PointType>::GetPoint(const double dt,
                                                              Point& p) const
{
  Polynomial::Point1d point1d;

  for (int dim=X; dim<kNumDim; ++dim) {
    polynomials_.at(dim).GetPoint(dt, point1d);
    p.p(dim) = point1d.x;
    p.v(dim) = point1d.xd;
    p.a(dim) = point1d.xdd;
    p.j(dim) = point1d.xddd;
  }

  return true;
}


} // namespace utils
} // namespace xpp
