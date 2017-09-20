/**
 @file    dynamic_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <Eigen/Dense>
#include <xpp/models/robot_model.h>

namespace xpp {
namespace opt {

RobotModel::RobotModel (int ee_count)
{
  for (int ee=0; ee<ee_count; ee++)
    ee_ids_.push_back(static_cast<EndeffectorID>(ee));

  kinematic_model_ = std::make_shared<KinematicModel>();
  kinematic_model_->nominal_stance_.SetCount(ee_count);

  contact_timings_ = ContactTimings(ee_count);
}


std::vector<EndeffectorID>
xpp::opt::RobotModel::GetEEIDs () const
{
  return ee_ids_;
}

std::vector<std::string>
xpp::opt::RobotModel::GetEndeffectorNames () const
{
  std::vector<std::string> names_;
  auto map_ee_to_id = ReverseMap(map_id_to_ee_);
  for (EndeffectorID ee : ee_ids_)
     names_.push_back(map_ee_to_id.at(ee));

  return names_;
}

std::vector<double>
RobotModel::GetNormalizedInitialTimings (EndeffectorID ee)
{
  NormalizeTimesToOne(ee);
  return contact_timings_.at(ee);
}

void
RobotModel::NormalizeTimesToOne(EndeffectorID ee)
{
  auto& v = contact_timings_.at(ee); // shorthand
  double total_time = std::accumulate(v.begin(), v.end(), 0.0);
  std::transform(v.begin(), v.end(), v.begin(),
                 [total_time](double t_phase){ return t_phase/total_time;});
}

RobotModel::~RobotModel ()
{
}

} /* namespace opt */
} /* namespace xpp */
