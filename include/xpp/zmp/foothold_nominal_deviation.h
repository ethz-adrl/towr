/**
 @file    foothold_nominal_deviation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class FootholdNominalDeviation {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;

  FootholdNominalDeviation ();
  virtual ~FootholdNominalDeviation () {}

  VectorXd DistanceToNominalStance(const ContinuousSplineContainer&,
                                   const SupportPolygonContainer&) const;

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_ */
