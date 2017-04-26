/**
@file    spliner_3d.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Creates 3 dimensional spline from start to end with duration T
 */

#include <cassert>

namespace xpp {
namespace opt {

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::PolynomialXd ()
{
}

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::PolynomialXd (double duration)
{
  SetBoundary(duration, PointT(), PointT());
}

template<typename PolynomialType, typename PointType>
PolynomialXd<PolynomialType, PointType>::~PolynomialXd ()
{
}

template<typename PolynomialType, typename PointType>
const double
PolynomialXd<PolynomialType, PointType>::GetDuration () const
{
  // all polynomials have same duration, so just return duration of X
  return polynomials_.at(X).GetDuration();
}

template<typename PolynomialType, typename PointType>
const double
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
                                                          const PointT& start,
                                                          const PointT& end)
{
  for (int dim=X; dim<kNumDim; ++dim)
    polynomials_.at(dim).SetBoundary(T, start.Get1d(dim), end.Get1d(dim));
}

template<typename PolynomialType, typename PointType>
PointType
PolynomialXd<PolynomialType, PointType>::GetPoint(const double dt) const
{
  PointT p;
  for (int dim=X; dim<kNumDim; ++dim)
    p.SetDimension(polynomials_.at(dim).GetPoint(dt), dim);

  return p;
}

template<typename PolynomialType, typename PointType>
const PolynomialType
PolynomialXd<PolynomialType, PointType>::GetDim (int dim) const
{
  return polynomials_.at(dim);
}

///////////////////////////////////////////////////////////////////////////////

template<typename TPolyXd>
double
PolyVecManipulation<TPolyXd>::GetTotalTime(
    const VecPolynomials& splines)
{
  double T = 0.0;
  for (const auto& s: splines)
    T += s.GetDuration();
  return T;
}

template<typename TPolyXd>
double
PolyVecManipulation<TPolyXd>::GetLocalTime(
    double t_global, const VecPolynomials& splines)
{
  int id_spline = GetPolynomialID(t_global,splines);

  double t_local = t_global;
  for (int id=0; id<id_spline; id++) {
    t_local -= splines.at(id).GetDuration();
  }

  return t_local;//-eps_; // just to never get value greater than true duration due to rounding errors
}

template<typename TPolyXd>
typename PolyVecManipulation<TPolyXd>::PointType
PolyVecManipulation<TPolyXd>::GetPoint(
    double t_global, const VecPolynomials& splines)
{
  int idx        = GetPolynomialID(t_global,splines);
  double t_local = GetLocalTime(t_global, splines);

  return splines.at(idx).GetPoint(t_local);
}

template<typename TPolyXd>
int
PolyVecManipulation<TPolyXd>::GetPolynomialID(
    double t_global, const VecPolynomials& splines)
{
  double eps = 1e-10; // double imprecision
  assert(t_global<=GetTotalTime(splines)+eps); // machine precision

   double t = 0;
   int i=0;
   for (const auto& s: splines) {
     t += s.GetDuration();

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }
   assert(false); // this should never be reached
}

} // namespace opt
} // namespace xpp
