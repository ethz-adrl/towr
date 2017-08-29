/**
 @file    endeffector_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 29, 2017
 @brief   Brief description
 */

#include <xpp/variables/endeffector_motion.h>

#include <string>

#include <xpp/variables/variable_names.h>

namespace xpp {
namespace opt {

EndeffectorMotion::EndeffectorMotion (EndeffectorID ee)
    :Component(-1, id::GetEEMotionId(ee))
{
}

EndeffectorMotion::~EndeffectorMotion ()
{
}

} /* namespace opt */
} /* namespace xpp */
