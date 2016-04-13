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

} /* namespace zmp */
} /* namespace xpp */
