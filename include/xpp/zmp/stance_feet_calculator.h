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
#include <xpp/zmp/zmp_spline.h>

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
  typedef std::vector<xpp::zmp::ZmpSpline> VecSpline;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef Eigen::Vector2d Vector2d;

  StanceFeetCalculator ();
  virtual ~StanceFeetCalculator ();

  void Update(const VecFoothold& start_stance, const VecFoothold& steps, const VecSpline& cog_spline);
  VecFoothold GetStanceFeetInBase(double t) const;

private:
  VecFoothold ConvertFeetToBase(VecFoothold ee_i, Vector2d cog_i) const;

  VecSpline cog_spline_;
  SupportPolygonContainer supp_polygon_container_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_ */
