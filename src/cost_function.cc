/*
 * cost_funtion.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/cost_function.h>
#include <array>

namespace xpp {
namespace zmp {

using utils::X; using utils::Y; using utils::Z;

CostFunction::CostFunction (const ContinuousSplineContainer& spline_structure)
{
  spline_structure_ = spline_structure;
  n_variables_ = spline_structure_.GetTotalFreeCoeff();
  cf_quadratic_ = CreateMinAccCostFunction();
}


double
CostFunction::EvalObjective(const Eigen::VectorXd& x) const
{
  // some aliases
  const Eigen::MatrixXd& M = cf_quadratic_.M;
  const Eigen::VectorXd& q = cf_quadratic_.v;

  assert(x.rows() == n_variables_);
  double obj_value = 0.0;
  obj_value = x.transpose() * M * x;
  obj_value += q.transpose() * x;

  return obj_value;
}


Eigen::VectorXd
CostFunction::EvalGradientOfObjective(const Eigen::VectorXd& x) const
{
  assert(x.rows() == n_variables_);
  return cf_quadratic_.M*x;
}


CostFunction::MatVec
CostFunction::CreateMinAccCostFunction() const
{
  std::array<double,2> weight = {1.0, 5.0}; // weight in x and y direction

  // total number of coefficients to be optimized
  int n_coeff = spline_structure_.GetTotalFreeCoeff();
  MatVec cf(n_coeff, n_coeff);

  for (const ZmpSpline& s : spline_structure_.splines_) {
    std::array<double,10> t_span = utils::cache_exponents<10>(s.duration_);

    for (int dim = X; dim <= Y; dim++) {
      const int a = ContinuousSplineContainer::Index(s.id_, dim, A);
      const int b = ContinuousSplineContainer::Index(s.id_, dim, B);
      const int c = ContinuousSplineContainer::Index(s.id_, dim, C);
      const int d = ContinuousSplineContainer::Index(s.id_, dim, D);

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
