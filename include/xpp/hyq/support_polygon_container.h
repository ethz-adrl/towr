/*
 * supp_triangle_container.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/hyq/support_polygon.h>

namespace xpp {
namespace hyq {

class SupportPolygonContainer
{
public:
  typedef std::vector<SupportPolygon> VecSupportPolygon;
  typedef SupportPolygon::VecFoothold VecFoothold;
  typedef xpp::utils::MatVec MatVec;


public:
  SupportPolygonContainer () {};
  virtual
  ~SupportPolygonContainer () {};

public:
  void Init(LegDataMap<Foothold> start_stance,
            const VecFoothold& footholds,
            const MarginValues& margins);


  VecSupportPolygon CreateSupportPolygons() const;
  Eigen::Vector2d GetCenterOfFinalStance() const;
  static bool Insert4LSPhase(LegID prev, LegID next);

  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;

  VecFoothold GetFootholds() const { return footholds_; };
  void SetFootholdsXY(size_t idx, double x, double y)
  {
    footholds_.at(idx).p.x() = x;
    footholds_.at(idx).p.y() = y;
  };

  LegDataMap<Foothold> start_stance_;
  MarginValues margins_;

private:
  VecFoothold footholds_;
  void CheckIfInitialized() const;

  SupportPolygon GetStancePolygon(const LegDataMap<Foothold>& stance) const;
  LegDataMap<Foothold> GetLastStance() const;
  bool initialized_ = false;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
