/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <xpp/utils/polynomial_3d.h>

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

template<typename SplineType>
void Spliner3d<SplineType>::SetBoundary(double T, const Point& start,
                                                  const Point& end)
{
	Polynomial::Point1d start_x, end_x;
	Polynomial::Point1d start_y, end_y;
	Polynomial::Point1d start_z, end_z;

	start_x.x   = start.p(X);  end_x.x   = end.p(X);
	start_x.xd  = start.v(X);  end_x.xd  = end.v(X);
	start_x.xdd = start.a(X);  end_x.xdd = end.a(X);

	start_y.x   = start.p(Y);  end_y.x   = end.p(Y);
	start_y.xd  = start.v(Y);  end_y.xd  = end.v(Y);
	start_y.xdd = start.a(Y);  end_y.xdd = end.a(Y);

	start_z.x   = start.p(Z);  end_z.x   = end.p(Z);
	start_z.xd  = start.v(Z);  end_z.xd  = end.v(Z);
	start_z.xdd = start.a(Z);  end_z.xdd = end.a(Z);


	splineX.SetBoundary(T, start_x, end_x);
	splineY.SetBoundary(T, start_y, end_y);
	splineZ.SetBoundary(T, start_z, end_z);
}

template<typename SplineType>
bool Spliner3d<SplineType>::GetPoint(const double dt, Point& p) const
{
  Polynomial::Point1d coord_result;

  splineX.GetPoint(dt, coord_result);
  p.p(X) = coord_result.x;
  p.v(X) = coord_result.xd;
  p.a(X) = coord_result.xdd;
  p.j(X) = coord_result.xddd;

  splineY.GetPoint(dt, coord_result);
  p.p(Y) = coord_result.x;
  p.v(Y) = coord_result.xd;
  p.a(Y) = coord_result.xdd;
  p.j(Y) = coord_result.xddd;

  splineZ.GetPoint(dt, coord_result);
  p.p(Z) = coord_result.x;
  p.v(Z) = coord_result.xd;
  p.a(Z) = coord_result.xdd;
  p.j(Z) = coord_result.xddd;

  return true;
}

} // namespace utils
} // namespace xpp
