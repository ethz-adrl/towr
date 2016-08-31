/**
 @file    support_polygon_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declares the SupportPolygonContainer class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/hyq/support_polygon.h>
#include <xpp/zmp/com_motion.h>

#include <functional> // std::function

namespace xpp {
namespace hyq {

/** @brief Hold the support polygons created by the contacts with the environment.
  *
  * This class is responsible for all tasks that turn footholds into support
  * polygons. Since support polygons are paired with phases of the CoM motion,
  * this class also depends on com_motion.
  */
class SupportPolygonContainer
{
public:
  typedef std::vector<SupportPolygon> VecSupportPolygon;
  typedef SupportPolygon::VecFoothold VecFoothold;
  typedef std::vector<xpp::hyq::LegID> VecLegID;
  typedef std::vector<SupportPolygon::VecSuppLine> VecVecSuppLine;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::zmp::ComMotion ComMotion;
  typedef xpp::utils::Coords3D Coords;


public:
  SupportPolygonContainer () {};
  virtual ~SupportPolygonContainer () {};

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

  Eigen::Vector2d GetCenterOfFinalStance() const;
  VecFoothold GetStanceDuring(int step) const;
  VecFoothold GetStanceAfter(int n_steps) const;
  VecFoothold GetFootholds() const { return footholds_; };

  /** Position where the foothold is stored in the optimization variables.
   *
   *  u = [x0 y0 x1 y1 ... xN yN]
   */
  int Index(int foothold_id, Coords dim) const;
  int GetTotalFreeCoeff() const;


  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;
  VecFoothold GetFinalFootholds() const ;

  int GetNumberOfSteps() const { return footholds_.size(); };
  void SetFootholdsXY(const StdVecEigen2d& footholds_xy);

  VecFoothold GetStartStance() const {return start_stance_;};
  Foothold GetStartFoothold(LegID leg) const;


  /** @brief First step is considered step=0. */
  LegID GetLegID(int step) const { return footholds_.at(step).leg; };


  VecSupportPolygon GetSupportPolygons() const {return support_polygons_;};

  VecSupportPolygon AssignSupportPolygonsToPhases(const ComMotion&) const;
  VecVecSuppLine GetActiveConstraintsForEachPhase(const ComMotion&) const;

  /** @brief returns the foothold sequence, but each leg is initialized to start stance xy */
  Eigen::VectorXd GetFootholdsInitializedToStart() const;


private:

  VecFoothold footholds_;
  VecSupportPolygon support_polygons_;
  MarginValues margins_;
  VecFoothold start_stance_;

  VecSupportPolygon CreateSupportPolygons(const VecFoothold& footholds) const;
  void CheckIfInitialized() const;

  void ModifyFootholds (VecFoothold& footholds, std::function<void (Foothold&, int)>) const;

  SupportPolygon GetStancePolygon(const VecFoothold& footholds) const;
  bool initialized_ = false;
  friend class SuppPolygonContainerTest_CreateSupportPolygons_Test;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
