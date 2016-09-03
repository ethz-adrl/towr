/**
 @file    foothold_nominal_deviation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_

#include "com_motion.h"
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class FootholdNominalDeviation {
public:

  typedef ComMotion::PtrS ComMotionPtr;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;

  FootholdNominalDeviation ();
  virtual ~FootholdNominalDeviation () {}

  StdVecEigen2d GetFeetInBase(const ComMotionPtr&,
                              const SupportPolygonContainer&,
                              StdVecEigen2d& nominal_foothold_b_) const;

private:
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_ */
