/**
 @file    stance_feet_calculator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_

#include <xpp/hyq/foothold.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace zmp {

/** @brief Determines the relationship between stance legs and CoG position.
  *
  * This class is responsible for supplying the position of the stance legs
  * relative to the CoG.
  */
class StanceFeetCalculator {
public:
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef ComMotion::PtrS ComSplinePtr;
  typedef Eigen::Vector3d Vector3d;

  StanceFeetCalculator ();
  virtual ~StanceFeetCalculator ();

  void Update(const VecFoothold& start_stance, const VecFoothold& steps,
              const ComSplinePtr& cog_spline, double robot_height);
  VecFoothold GetStanceFeetInBase(double t) const;

private:
  VecFoothold ConvertFeetToBase(const VecFoothold& ee_i, const Vector3d& cog_i) const;
  bool AreSame(double time_1, double time_2) const;

  ComSplinePtr com_motion_;
  SupportPolygonContainer supp_polygon_container_;
  double robot_height_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_ */
