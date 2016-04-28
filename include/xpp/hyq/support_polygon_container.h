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
  typedef xpp::hyq::LegID LegID;


public:
  SupportPolygonContainer () {};
  virtual
  ~SupportPolygonContainer () {};

public:
  void Init(const VecFoothold& start_stance,
            const VecFoothold& footholds, // remove this, not really neccessary
            const std::vector<LegID>& step_sequence,
            const MarginValues& margins = SupportPolygon::GetZeroMargins());


  Eigen::Vector2d GetCenterOfFinalStance() const;
  static bool Insert4LSPhase(LegID prev, LegID next);
  VecFoothold GetStanceDuring(int step) const;
  VecFoothold GetStanceAfter(int n_steps) const;


  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;
  VecFoothold GetFinalFootholds() const ;

//  VecFoothold GetFootholds() const { return footholds_; };
  int GetNumberOfSteps() const { return footholds_.size(); };
  void SetFootholdsXY(int idx, double x, double y);

  VecFoothold GetStartStance() const {return start_stance_;};
  /**
   * First step is considered step=0.
   */
  LegID GetLegID(int step) const { return step_sequence_.at(step); };


  VecSupportPolygon GetSupportPolygons() const {return support_polygons_;};


private:

  VecFoothold footholds_; // fixme remove this
  std::vector<LegID> step_sequence_;
  VecSupportPolygon support_polygons_; // fixme remove this
  MarginValues margins_;
  VecFoothold start_stance_;

  VecSupportPolygon CreateSupportPolygons(const VecFoothold& footholds) const;
  void CheckIfInitialized() const;

  SupportPolygon GetStancePolygon(const VecFoothold& footholds) const;
  bool initialized_ = false;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
