/**
 @file    parametrization.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#include <xpp/parametrization.h>

namespace xpp {
namespace opt {

Parametrization::Parametrization (const std::string& id)
{
  id_ = id;
}

Parametrization::~Parametrization ()
{
}

int
Parametrization::GetOptVarCount () const
{
  return GetOptimizationParameters().rows();
}

std::string
Parametrization::GetID () const
{
  return id_;
}

} /* namespace opt */
} /* namespace xpp */
