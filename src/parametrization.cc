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
  return GetVariables().rows();
}

std::string
Parametrization::GetId () const
{
  return id_;
}

VecBound
Parametrization::GetBounds () const
{
  // default value if user hasn't set anything
  if (bounds_.empty())
    SetAllBounds(kNoBound_);

  return bounds_;
}

void
Parametrization::SetAllBounds (const Bound& bound) const
{
  bounds_ = VecBound(GetOptVarCount(), bound);
}

} /* namespace opt */
} /* namespace xpp */

