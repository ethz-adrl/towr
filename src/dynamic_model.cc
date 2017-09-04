/**
 @file    dynamic_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <xpp/dynamic_model.h>

#include <Eigen/Dense>

namespace xpp {
namespace opt {

DynamicModel::DynamicModel (int ee_count)
{
  for (int ee=0; ee<ee_count; ee++)
    ee_ids_.push_back(static_cast<EndeffectorID>(ee));

  nominal_stance_.SetCount(ee_count);
  contact_timings_ = ContactTimings(ee_count);
}

void
DynamicModel::SetCurrent (const ComPos& com_pos, const EELoad& ee_force,
                          const EEPos& ee_pos)
{
  com_pos_  = com_pos;
  ee_force_ = ee_force;
  ee_pos_   = ee_pos;
}

std::vector<EndeffectorID>
xpp::opt::DynamicModel::GetEEIDs () const
{
  return ee_ids_;
}

std::vector<std::string>
xpp::opt::DynamicModel::GetEndeffectorNames () const
{
  std::vector<std::string> names_;
  auto map_ee_to_id = ReverseMap(map_id_to_ee_);
  for (EndeffectorID ee : ee_ids_)
     names_.push_back(map_ee_to_id.at(ee));

  return names_;
}

std::vector<double>
DynamicModel::GetNormalizedInitialTimings (EndeffectorID ee) const
{
  // normalize vector, so equals 1.0
  auto v = contact_timings_.at(ee);
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  std::transform(v.begin(), v.end(), v.begin(), [sum](auto& e){ return e/sum;});
  return v;
}

DynamicModel::~DynamicModel ()
{
}

} /* namespace opt */
} /* namespace xpp */
