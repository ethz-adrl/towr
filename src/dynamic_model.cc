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

DynamicModel::~DynamicModel ()
{
}

} /* namespace opt */
} /* namespace xpp */

