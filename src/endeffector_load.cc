/**
 @file    endeffector_load.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffector_load.h>

namespace xpp {
namespace opt {

EndeffectorLoad::EndeffectorLoad () : Parametrization("convexity_lambdas")
{
}

EndeffectorLoad::~EndeffectorLoad ()
{
}

void
EndeffectorLoad::Init (const EndeffectorsMotion ee_motion, double dt, double T)
{
  dt_ = dt;
  n_ee_ = ee_motion.GetNumberOfEndeffectors();
  int num_parameters = n_ee_ * GetSegment(T);
  lambdas_ = VectorXd::Zero(num_parameters);
}

void
EndeffectorLoad::SetOptimizationParameters (const VectorXd& x)
{
  lambdas_ = x;
}

EndeffectorLoad::VectorXd
EndeffectorLoad::GetOptimizationParameters () const
{
  return lambdas_;
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValues (double t) const
{
  int k = GetSegment(t);
  return GetLoadValuesIdx(k);
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValuesIdx (int k_curr) const
{
  LoadParams load(n_ee_);
  for (auto ee : load.GetEEsOrdered())
    load.At(ee) = lambdas_(IndexDiscrete(k_curr,ee));

  return load;
}

int
EndeffectorLoad::Index (double t, EndeffectorID ee) const
{
  int k = GetSegment(t);
  return IndexDiscrete(k, ee);
}

int
EndeffectorLoad::IndexDiscrete (int k_curr, EndeffectorID ee) const
{
  return n_ee_*k_curr + ee;
}

int
EndeffectorLoad::GetNumberOfSegments () const
{
  return GetOptVarCount()/n_ee_;
}

double
EndeffectorLoad::GetTStart (int node) const
{
  return node*dt_;
}

int
EndeffectorLoad::GetSegment (double t) const
{
  return floor(t/dt_);
}

} /* namespace opt */
} /* namespace xpp */

