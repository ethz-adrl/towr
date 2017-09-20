/**
 @file    dynamic_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#include <xpp/models/dynamic_model.h>

namespace xpp {
namespace opt {

DynamicModel::DynamicModel(double mass)
{
  m_ = mass;
  g_ = 9.80665;
}

void
DynamicModel::SetCurrent (const ComPos& com_pos, const EELoad& ee_force,
                          const EEPos& ee_pos)
{
  com_pos_  = com_pos;
  ee_force_ = ee_force;
  ee_pos_   = ee_pos;
}

double
DynamicModel::GetStandingZForce () const
{
  return m_*g_/ee_pos_.GetCount();
}

DynamicModel::~DynamicModel()
{
  // TODO Auto-generated destructor stub
}

} /* namespace opt */
} /* namespace xpp */
