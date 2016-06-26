/*
 * supp_triangle_container.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/zmp/zmp_spline.h>
#include <xpp/hyq/support_polygon.h>

namespace xpp {
namespace hyq {

/** @brief Hold the support polygons created by the contacts with the environment.
  */
class SupportPolygonContainer
{
public:
  typedef std::vector<SupportPolygon> VecSupportPolygon;
  typedef SupportPolygon::VecFoothold VecFoothold;
  typedef std::vector<xpp::hyq::LegID> VecLegID;
  typedef xpp::utils::MatVec MatVec;
  typedef std::vector<xpp::zmp::ZmpSpline> VecZmpSpline;
  typedef std::vector<SupportPolygon::VecSuppLine> VecVecSuppLine;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;


public:
  SupportPolygonContainer () {};
  virtual
  ~SupportPolygonContainer () {};

public:

  /** @brief Initializes with the info needed for the QP optimizer, which includes
    * foothold locations.
    *
    * @param start_stance the feet that are initial in contact with the environment
    * @param footholds the steps to take
    * @param margins how much to shrink the support polygon
    */
  void Init(const VecFoothold& start_stance,
            const VecFoothold& footholds,
            const MarginValues& margins = SupportPolygon::GetZeroMargins());

  /** @brief Initializes with the info needed for the NLP optimizer, that only needs to
    * know the step sequence, and will optimize the foothold locations.
    *
    * Each foothold is initialized to x=y=z=0.
    *
    * @param start_stance the feet that are initial in contact with the environment
    * @param footholds the order of steps to take
    * @param margins how much to shrink the support polygon
    */
  void Init(const VecFoothold& start_stance,
            const VecLegID& step_sequence,
            const MarginValues& margins = SupportPolygon::GetZeroMargins());

  void Init(const VecFoothold& start_stance);

  Eigen::Vector2d GetCenterOfFinalStance() const;
//  static bool Insert4LSPhase(LegID prev, LegID next);
  VecFoothold GetStanceDuring(int step) const;
  VecFoothold GetStanceAfter(int n_steps) const;


  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;
  VecFoothold GetFinalFootholds() const ;

//  VecFoothold GetFootholds() const { return footholds_; };
  int GetNumberOfSteps() const { return footholds_.size(); };
  void SetFootholdsXY(int idx, double x, double y);

  VecFoothold GetStartStance() const {return start_stance_;};
  Foothold GetStartFoothold(LegID leg) const;


  /** @brief First step is considered step=0. */
  LegID GetLegID(int step) const { return footholds_.at(step).leg; };


  VecSupportPolygon GetSupportPolygons() const {return support_polygons_;};

  VecSupportPolygon AssignSupportPolygonsToSplines(const VecZmpSpline&) const;
  VecVecSuppLine GetActiveConstraintsForEachStep(const VecZmpSpline&) const;

  /** @brief returns the foothold sequence, but each leg is initialized to start stance xy */
  StdVecEigen2d GetFootholdsInitializedToStart() const;


private:

  VecFoothold footholds_;
  VecSupportPolygon support_polygons_;
  MarginValues margins_;
  VecFoothold start_stance_;

  VecSupportPolygon CreateSupportPolygons(const VecFoothold& footholds) const;
  void CheckIfInitialized() const;

  SupportPolygon GetStancePolygon(const VecFoothold& footholds) const;
  bool initialized_ = false;
  friend class SuppPolygonContainerTest_CreateSupportPolygons_Test;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
