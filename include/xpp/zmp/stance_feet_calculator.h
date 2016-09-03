/**
 @file    stance_feet_calculator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_
#define XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_

#include <xpp/hyq/leg_data_map.h>

#include <Eigen/Dense>
#include <Eigen/StdVector> // for std::vector<Eigen:...>
#include <vector>
#include <memory>

namespace xpp {

namespace hyq {
class Foothold;
class SupportPolygonContainer;
}

namespace zmp {

class ComMotion;

/** @brief Determines the relationship between stance legs and CoG position.
  *
  * This class is responsible for supplying the position of the stance legs
  * relative to the CoG.
  */
class StanceFeetCalculator {
public:
  using StanceFootholds = std::vector<xpp::hyq::Foothold>;
  using StanceVecT      = std::vector<StanceFootholds>;
  using PositionVecT    = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;

  using MotionCoeff  = Eigen::VectorXd;
  using SupportPolygonContainer = xpp::hyq::SupportPolygonContainer;
  using ComMotionPtrU = std::unique_ptr<ComMotion>;
  using ComSuppPolyPtrU = std::unique_ptr<SupportPolygonContainer>;


  StanceFeetCalculator ();
  StanceFeetCalculator (const std::vector<double>& times, const ComMotion&,
                        const SupportPolygonContainer&);

  virtual ~StanceFeetCalculator ();

  void Init(const std::vector<double>& times, const ComMotion&,
            const SupportPolygonContainer&);

  void Update(const MotionCoeff&, const PositionVecT&);



  // new and improved functions
  PositionVecT CalculateComPostionInWorld() const;
  StanceVecT GetStanceFootholdsInWorld() const;



  struct ContactInfo {
    ContactInfo(double time, int id, xpp::hyq::LegID leg)
        :time_(time), foothold_id_(id), leg_(leg) {};

    double time_;
    double foothold_id_;
    xpp::hyq::LegID leg_;
  };

  // this info will never change! fixed once initially
  // should move to higher level class
  std::vector<ContactInfo> GetContactInfoVec() const;



//  void Update(const StanceFootholds& start_stance, const StanceFootholds& steps,
//              const ComSplinePtr& cog_spline, double robot_height);
//  StanceFootholds GetStanceFeetInBase(double t) const;

  std::vector<double> times_;
private:

  ComMotionPtrU com_motion_;
  ComSuppPolyPtrU foothold_container_;

//  double robot_height_;
//  StanceFootholds ConvertFeetToBase(const StanceFootholds& ee_i, const PosXYZ& cog_i) const;
//  bool AreSame(double time_1, double time_2) const;


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_ */
