/**
 @file    dynamic_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include "xpp/opt/dynamic_model.h"

namespace xpp {
namespace opt {

DynamicModel::DynamicModel ()
{
  // TODO Auto-generated constructor stub

}

DynamicModel::~DynamicModel ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicModel::SetCurrent (const ComPos& com_pos, const EELoad& ee_load,
                          const EEPos& ee_pos)
{
  com_pos_ = com_pos;
  ee_load_ = ee_load;
  ee_pos_  = ee_pos;
}

std::vector<EndeffectorID>
xpp::opt::DynamicModel::GetEEIDs () const
{
  return ee_load_.GetEEsOrdered();
}

} /* namespace opt */
} /* namespace xpp */

