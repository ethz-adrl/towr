/*
 * problem_specification.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: winklera
 */

#include <xpp/zmp/problem_specification.h>

namespace xpp {
namespace zmp {

ProblemSpecification::ProblemSpecification (const SupportPolygonContainer& supp_poly_container,
                                            const ContinuousSplineContainer& cog_spline_container)
    :planned_footholds_(supp_poly_container.GetFootholds())
{
  zmp_spline_container_    = cog_spline_container;
  supp_polygon_container_  = supp_poly_container;
}

ProblemSpecification::~ProblemSpecification ()
{
  // TODO Auto-generated destructor stub
}


Eigen::VectorXd
ProblemSpecification::DistanceFootFromPlanned(const StdVecEigen2d& footholds) const
{
  // constraints on the footsteps
  std::vector<int> fixed_dim = {xpp::utils::X, xpp::utils::Y};
  Eigen::VectorXd g(fixed_dim.size()*footholds.size());
  int c=0;

  for (uint i=0; i<footholds.size(); ++i) {
    xpp::hyq::Foothold f = planned_footholds_.at(i);

    // fix footholds in x and y direction
    for (int dim : fixed_dim)
      g(c++) = footholds.at(i)(dim) - f.p(dim);
  }

  return g;
}


Eigen::VectorXd
ProblemSpecification::DistanceFootToNominal(const VectorXd& x_coeff,
                                            const StdVecEigen2d& footholds) const
{
  double x_nominal_b = 0.3; // 0.4
  double y_nominal_b = 0.3; // 0.4

  xpp::hyq::LegDataMap<Vector2d> B_r_BaseToNominal;
  B_r_BaseToNominal[hyq::LF] <<  x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RF] <<  x_nominal_b, -y_nominal_b;
  B_r_BaseToNominal[hyq::LH] << -x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RH] << -x_nominal_b, -y_nominal_b;


  double dt = 0.3; //zmp_spline_container_.dt_;
  double T  = zmp_spline_container_.GetTotalTime();
  int N     = std::ceil(T/dt);
  int approx_n_constraints = 4*N*2; // 3 or 4 legs in contact at every discrete time, 2 b/c x-y
  std::vector<double> g_vec;
  g_vec.reserve(approx_n_constraints);


  // TODO check if coefficients are already updated with optimized ones
  // to save computation time
  // maybe call "UpdateOpt.." instead of "Add.."
//  zmp_spline_container_.AddOptimizedCoefficients(x_coeff);
  ContinuousSplineContainer::Splines updated_splines = zmp_spline_container_.splines_;
  zmp_spline_container_.AddOptimizedCoefficients(x_coeff, updated_splines);
  double t=0.0;
  do {
    // know legs in contact at each step
    VecFoothold stance_legs;
    int step = zmp_spline_container_.GetStep(t);
    stance_legs = supp_polygon_container_.GetStanceDuring(step);

    xpp::utils::Point2d cog_xy;
    SplineContainer::GetCOGxy(t, cog_xy, updated_splines);

    // calculate distance to base for every stance leg
    // restrict to quadrants
    for (const hyq::Foothold& f : stance_legs) {

      // base to foot
      Eigen::Vector2d r_BF = f.p.segment<2>(0) - cog_xy.p;

      // current foot to nominal
      Eigen::Vector2d r_FC = -r_BF + B_r_BaseToNominal[f.leg];

      // don't use std::fabs() or std::pow() since you loose information about sign
      // this information help the optimizer to find the correct gradient.
      g_vec.push_back(r_FC.x());
      g_vec.push_back(r_FC.y());
    }
    t += dt;

  } while(t < T);

  return Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
}




} /* namespace zmp */
} /* namespace xpp */
