/**
 @file    support_polygon_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declares the SupportPolygonContainer class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/hyq/support_polygon.h>
#include <xpp/utils/eigen_std_conversions.h>
#include <xpp/utils/cartesian_declarations.h>
#include <functional> // std::function

namespace xpp {

namespace opt {
class PhaseInfo;
}

namespace hyq {

/** @brief Hold the support polygons created by the contacts with the environment.
  *
  * This class is responsible for all tasks that turn footholds into support
  * polygons. Since support polygons are paired with phases of the CoM motion,
  * this class also depends on com_motion.
  */
class SupportPolygonContainer {
public:
  typedef std::vector<SupportPolygon> VecSupportPolygon;
  typedef SupportPolygon::VecFoothold VecFoothold;
  typedef std::vector<xpp::hyq::LegID> VecLegID;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef std::vector<xpp::opt::PhaseInfo> PhaseInfoVec;
  typedef xpp::utils::Coords3D Coords;
  using PosXY = Eigen::Vector2d;

  SupportPolygonContainer () {};
  virtual ~SupportPolygonContainer () {};

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

  void Init(const VecLegID& start_stance,
            const VecLegID& step_sequence,
            const MarginValues& margins = SupportPolygon::GetZeroMargins());

  Eigen::Vector2d GetCenterOfFinalStance() const;
  VecFoothold GetStanceDuring(int step) const;
  VecFoothold GetStanceAfter(int n_steps) const;
  VecFoothold GetFootholdsInWorld() const { return footholds_I_; };

  /** Position where the foothold is stored in the optimization variables.
   *
   *  u = [x0 y0 x1 y1 ... xN yN]
   */
  static int Index(int foothold_id, Coords dim);
  int GetTotalFreeCoeff() const;

  SupportPolygon GetStartPolygon() const;
  SupportPolygon GetFinalPolygon() const;
  VecFoothold GetFinalFootholds() const ;

  /** Only those footholds that are not the start stance
   */
  VecFoothold GetFinalFreeFootholds() const ;

  int GetNumberOfSteps() const { return footholds_I_.size(); };
  void SetFootholdsXY(const StdVecEigen2d& footholds_xy);

  VecFoothold GetStartStance() const {return start_stance_;};
  Foothold GetStartFoothold(LegID leg) const;


  /** @brief First step is considered step=0. */
  LegID GetLegID(int step) const { return footholds_I_.at(step).leg; };


  VecSupportPolygon GetSupportPolygons() const {return support_polygons_;};
  VecSupportPolygon AssignSupportPolygonsToPhases(const PhaseInfoVec&) const;


  /** @brief returns the foothold sequence, but each leg is initialized to start stance xy */
  // mpc remove this function
  Eigen::VectorXd GetFootholdsInitializedToStart() const;

  static bool DisJointSupportPolygons(LegID prev, LegID next);
  VecFoothold footholds_I_; // mpc make private again
private:

  VecSupportPolygon support_polygons_;
  MarginValues margins_;
  VecFoothold start_stance_;

  VecSupportPolygon CreateSupportPolygons(const VecFoothold& footholds) const;
  void ModifyFootholds (VecFoothold& footholds, std::function<void (Foothold&, int)>) const;

  SupportPolygon GetStancePolygon(const VecFoothold& footholds) const;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
