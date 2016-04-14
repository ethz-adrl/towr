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
ProblemSpecification::FixFootholdPosition(const StdVecEigen2d& footholds) const
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

} /* namespace zmp */
} /* namespace xpp */
