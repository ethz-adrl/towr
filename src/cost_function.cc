/*
 * cost_function1.cc
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#include <xpp/zmp/cost_function.h>

namespace xpp {
namespace zmp {


CostFunction::CostFunction(const ContinuousSplineContainer& spline_structure,
                           const SupportPolygonContainer& supp_polygon_container,
                           const NlpStructure& nlp_structure)
    : EigenNumDiffFunctord(nlp_structure.GetOptimizationVariableCount(),1),
      ProblemSpecification(supp_polygon_container,spline_structure),
      cf_(CreateMinAccCostFunction(spline_structure)),
      nlp_structure_(nlp_structure)
{};


double
CostFunction::EvalCostFunction(const InputType& x) const
{
  double obj_value = 0.0;
  obj_value += MinimizeAcceleration(nlp_structure_.ExtractSplineCoefficients(x));
//  obj_value += PenalizeFootholdFromPlanned(nlp_structure_.ExtractFootholds(x));

  return obj_value;
}


double
CostFunction::MinimizeAcceleration(const VectorXd& x_coeff) const
{
  double obj_value = 0.0;
  obj_value += x_coeff.transpose() * cf_.M * x_coeff;
  obj_value += cf_.v.transpose() * x_coeff;
  return obj_value;
}


double
CostFunction::PenalizeFootholdFromPlanned(const StdVecEigen2d& footholds) const
{
  Eigen::VectorXd g = DistanceFootFromPlanned(footholds);


//  double obj_value = 0.0;
//  for (const Vector2d& f : footholds) {
//    obj_value += f.x();
//  }
  return g.norm();
}


CostFunction::MatVec
CostFunction::CreateMinAccCostFunction(const ContinuousSplineContainer& spline_structure)
{
  using utils::X; using utils::Y;
  std::array<double,2> weight = {1.0, 3.0}; // weight in x and y direction

  // total number of coefficients to be optimized
  int n_coeff = spline_structure.GetTotalFreeCoeff();
  MatVec cf(n_coeff, n_coeff);

  for (const ZmpSpline& s : spline_structure.splines_) {
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
