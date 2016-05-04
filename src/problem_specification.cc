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
    /*:planned_footholds_(supp_poly_container.GetFootholds())*/
{
  zmp_spline_container_    = cog_spline_container;
  supp_polygon_container_  = supp_poly_container;
}

ProblemSpecification::~ProblemSpecification ()
{
  // TODO Auto-generated destructor stub
}


void
ProblemSpecification::UpdateCurrentState(const VectorXd& x_coeff, const StdVecEigen2d& footholds)
{
  zmp_spline_container_.AddOptimizedCoefficients(x_coeff);
  for (uint i=0; i<footholds.size(); ++i)
    supp_polygon_container_.SetFootholdsXY(i,footholds.at(i).x(), footholds.at(i).y());
}


//Eigen::VectorXd
//ProblemSpecification::DistanceFootFromPlanned(const StdVecEigen2d& footholds) const
//{
//  // constraints on the footsteps
//  std::vector<int> fixed_dim = {xpp::utils::X, xpp::utils::Y};
//  Eigen::VectorXd g(fixed_dim.size()*footholds.size());
//  int c=0;
//
//  for (uint i=0; i<footholds.size(); ++i) {
//    xpp::hyq::Foothold f = planned_footholds_.at(i);
//
//    // fix footholds in x and y direction
//    for (int dim : fixed_dim)
//      g(c++) = footholds.at(i)(dim) - f.p(dim);
//  }
//
//  return g;
//}


Eigen::VectorXd
ProblemSpecification::DistanceSquareFootToGapboarder(const StdVecEigen2d& footholds,
                                               double gap_center_x,
                                               double gap_width_x) const
{
  std::vector<double> g_vec;
//  xpp::utils::Ellipse ellipse(gap_depth, gap_width, x_center, y_center);

  for (const Vector2d& f : footholds)
  {
//    g_vec.push_back(ellipse.DistanceToEdge(f.p.x(), f.p.y()));
    double foot_to_center = f.x()-gap_center_x;
    double minimum_to_center = gap_width_x/2.0;
    double foot_to_boarder_square = std::pow(foot_to_center,2) - std::pow(minimum_to_center,2);
    g_vec.push_back(foot_to_boarder_square);
  }

  return Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
}



Eigen::VectorXd
ProblemSpecification::DistanceFootToNominalStance(const SupportPolygonContainer& supp_polygon_container,
                                                  const ContinuousSplineContainer& zmp_spline_container) const
{

  const double x_nominal_b = 0.3; // 0.4
  const double y_nominal_b = 0.3; // 0.4

  xpp::hyq::LegDataMap<Vector2d> B_r_BaseToNominal;
  B_r_BaseToNominal[hyq::LF] <<  x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RF] <<  x_nominal_b, -y_nominal_b;
  B_r_BaseToNominal[hyq::LH] << -x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RH] << -x_nominal_b, -y_nominal_b;

  int N     = zmp_spline_container.GetTotalNodes();
  int approx_n_constraints = 4*N*2; // 3 or 4 legs in contact at every discrete time, 2 b/c x-y
  std::vector<double> g_vec;
  g_vec.reserve(approx_n_constraints);

  std::vector<xpp::hyq::SupportPolygon> suppport_polygons =
      supp_polygon_container.CreateSupportPolygonsWith4LS(zmp_spline_container.GetSplines());


  for (double t : zmp_spline_container.GetDiscretizedGlobalTimes()) {

    // get legs in contact at each step
    VecFoothold stance_legs;
    int id = zmp_spline_container.GetSplineID(t);
    stance_legs = suppport_polygons.at(id).footholds_;

    xpp::utils::Point2d cog_xy = zmp_spline_container.GetCOGxy(t);

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
  }

  return Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
}




} /* namespace zmp */
} /* namespace xpp */
