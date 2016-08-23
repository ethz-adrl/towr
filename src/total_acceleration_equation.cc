/**
 @file    total_acceleration_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/total_acceleration_equation.h>

namespace xpp {
namespace zmp {

TotalAccelerationEquation::TotalAccelerationEquation (
    const ComSpline4& splines)
    :splines_(splines)
{}

TotalAccelerationEquation::MatVec
TotalAccelerationEquation::BuildLinearEquation () const
{
  using namespace xpp::utils::coords_wrapper;

  std::array<double,2> weight = {1.0, 3.0}; // weight in x and y direction

  // total number of coefficients to be optimized
  int n_coeff = splines_.GetTotalFreeCoeff();
  MatVec cf(n_coeff, n_coeff);

  for (const ZmpSpline& s : splines_.GetSplines()) {
    std::array<double,8> t_span = utils::cache_exponents<8>(s.GetDuration());

    for (const Coords3D dim : Coords2DArray) {
      const int a = ComSpline4::Index(s.GetId(), dim, A);
      const int b = ComSpline4::Index(s.GetId(), dim, B);
      const int c = ComSpline4::Index(s.GetId(), dim, C);
      const int d = ComSpline4::Index(s.GetId(), dim, D);

      // for explanation of values see M.Kalakrishnan et al., page 248
      // "Learning, Planning and Control for Quadruped Robots over challenging
      // Terrain", IJRR, 2010
      cf.M(a, a) = 400.0 / 7.0      * t_span[7] * weight[dim];
      cf.M(a, b) = 40.0             * t_span[6] * weight[dim];
      cf.M(a, c) = 120.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(a, d) = 10.0             * t_span[4] * weight[dim];
      cf.M(b, b) = 144.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(b, c) = 18.0             * t_span[4] * weight[dim];
      cf.M(b, d) = 8.0              * t_span[3] * weight[dim];
      cf.M(c, c) = 12.0             * t_span[3] * weight[dim];
      cf.M(c, d) = 6.0              * t_span[2] * weight[dim];
      cf.M(d, d) = 4.0              * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < cf.M.cols(); ++c)
        for (int r = c + 1; r < cf.M.rows(); ++r)
          cf.M(r, c) = cf.M(c, r);
    }
  }

  return cf;
}

} /* namespace zmp */
} /* namespace xpp */
