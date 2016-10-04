/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <xpp/utils/spliner_3d.h>

namespace xpp {
namespace utils {

template<typename SplineType, size_t N_DIM>
void
Spliner2d<SplineType, N_DIM>::SetDuration (double _duration)
{
  for (int dim=X; dim<kNumDim; ++dim)
    polynomials_.at(dim).duration = _duration;
}

template<typename SplineType, size_t N_DIM>
typename Spliner2d<SplineType, N_DIM>::Vector2d
Spliner2d<SplineType, N_DIM>::GetState (MotionDerivative pos_vel_acc_jerk, double t) const
{
  Point p;
  GetPoint(t, p);
  return p.GetByIndex(pos_vel_acc_jerk);
}

template<typename SplineType, size_t N_DIM>
double
Spliner2d<SplineType, N_DIM>::GetCoefficient (int dim, SplineCoeff coeff) const
{
  return polynomials_.at(dim).c[coeff];
}

template<typename SplineType, size_t N_DIM>
void
Spliner2d<SplineType, N_DIM>::SetCoefficients (int dim, SplineCoeff coeff, double value)
{
  polynomials_.at(dim).c[coeff] = value;
}

template<typename SplineType, size_t N_DIM>
void Spliner2d<SplineType, N_DIM>::SetBoundary(double T, const Point& start,
                                                  const Point& end)
{
  Spliner::Point1d _start[kNumDim];
  Spliner::Point1d _end[kNumDim];

  for (int dim=X; dim<kNumDim; ++dim) {
    // convert data types
    _start[dim].x   = start.p(X);  _end[dim].x   = end.p(X);
    _start[dim].xd  = start.v(X);  _end[dim].xd  = end.v(X);
    _start[dim].xdd = start.a(X);  _end[dim].xdd = end.a(X);

    polynomials_.at(dim).SetBoundary(T, _start[dim], _end[dim]);
  }
}

template<typename SplineType, size_t N_DIM>
bool Spliner2d<SplineType, N_DIM>::GetPoint(const double dt, Point& p) const
{
  Spliner::Point1d coord_result;

  for (int dim=X; dim<kNumDim; ++dim) {
    polynomials_[dim].GetPoint(dt, coord_result);
    p.p(dim) = coord_result.x;
    p.v(dim) = coord_result.xd;
    p.a(dim) = coord_result.xdd;
    p.j(dim) = coord_result.xddd;
  }

  return true;
}

template<typename SplineType>
void Spliner3d<SplineType>::SetBoundary(double T, const Point& start,
                                                  const Point& end)
{
	Spliner::Point1d start_x, end_x;
	Spliner::Point1d start_y, end_y;
	Spliner::Point1d start_z, end_z;

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
  Spliner::Point1d coord_result;

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
