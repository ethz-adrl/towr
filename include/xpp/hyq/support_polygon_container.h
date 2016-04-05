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


  Eigen::Vector2d GetCenterOfFinalStance() const;
  static bool Insert4LSPhase(LegID prev, LegID next);

  VecFoothold GetStanceDuring(int step) const;
  VecFoothold GetStanceAfter(int n_steps) const;

  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;

  VecFoothold GetFootholds() const { return footholds_; };
  void SetFootholdsXY(size_t idx, double x, double y);

  LegDataMap<Foothold> GetStartStance() const {return start_stance_;};


  VecSupportPolygon GetSupportPolygons() const {return support_polygons_;};



private:
  MarginValues margins_;
  LegDataMap<Foothold> start_stance_;
  VecFoothold footholds_;
  VecSupportPolygon support_polygons_;

  void CheckIfInitialized() const;
  VecSupportPolygon CreateSupportPolygons(const VecFoothold& footholds) const;

  SupportPolygon GetStancePolygon(const VecFoothold& footholds) const;
  bool initialized_ = false;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
