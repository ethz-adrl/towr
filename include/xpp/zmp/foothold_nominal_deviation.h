/**
 @file    foothold_nominal_deviation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_

// refactor   only use com_motion, shouldn't have to know about internal structure.
#include <xpp/hyq/support_polygon_container.h>
#include "com_spline.h"

namespace xpp {
namespace zmp {

class FootholdNominalDeviation {
public:

  typedef ComSpline::Ptr ComMotionPtr;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;

  FootholdNominalDeviation ();
  virtual ~FootholdNominalDeviation () {}

  StdVecEigen2d GetFeetInBase(const ComMotionPtr&,
                              const SupportPolygonContainer&,
                              StdVecEigen2d& nominal_foothold_b_) const;

//  StdVecEigen2d GetNominalInBase() const;

private:
//  StdVecEigen2d nominal_foothold_b_; ///< positions x-y of nominal position in base frame of that foothold  const double x_nominal_b = 0.3; // 0.4
//  const double x_nominal_b = 0.3; // 0.4
//  const double y_nominal_b = 0.3; // 0.4
//  xpp::hyq::LegDataMap<Eigen::Vector2d> B_r_BaseToNominal;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_ */
