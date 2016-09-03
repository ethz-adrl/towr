/**
 @file    stance_feet_calculator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_
#define XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_

#include <xpp/hyq/leg_data_map.h>
#include <vector>

namespace xpp {
namespace zmp {

class PhaseInfo;

/** @brief Determines the relationship between stance legs and CoG position.
  *
  * This class is responsible for supplying the position of the stance legs
  * relative to the CoG.
  */
class StanceFeetCalculator {
public:
  using LegIDVec = std::vector<xpp::hyq::LegID>;
  using PhaseVec = std::vector<PhaseInfo>;

  StanceFeetCalculator ();
  StanceFeetCalculator (const LegIDVec& start_legs, const LegIDVec& step_legs,
                        const PhaseVec& phases, double dt);
  virtual ~StanceFeetCalculator ();

  void Init(const LegIDVec& start_legs, const LegIDVec& step_legs,
            const PhaseVec& phases, double dt);

  struct ContactInfo {
    ContactInfo(double time, int id, xpp::hyq::LegID leg)
        :time_(time), foothold_id_(id), leg_(leg) {};

    double time_;
    double foothold_id_;
    xpp::hyq::LegID leg_;
  };

  std::vector<ContactInfo> GetContactInfoVec() const;

private:
  std::vector<ContactInfo> BuildContactInfoVec(double dt) const;
  std::vector<ContactInfo> contact_info_vec_;

  LegIDVec start_stance_;
  LegIDVec steps_;
  PhaseVec phases_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_ZMP_STANCE_FEET_CALCULATOR_H_ */
