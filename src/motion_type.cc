/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/opt/motion_type.h>

namespace xpp {
namespace opt {

MotionType::~MotionType ()
{
}

Walk::Walk()
{
  id_ = WalkID;
  swinglegs_per_phase_ = 1;
  t_phase_ = 0.5;
  max_step_length_ = 0.21;
}

Trott::Trott()
{
  id_ = TrottID;
  swinglegs_per_phase_ = 2;
  t_phase_ = 0.3;
  max_step_length_ = 0.15;
}

}
}

