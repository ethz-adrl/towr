/**
 @file    endeffector_load.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffector_load.h>

namespace xpp {
namespace opt {

EndeffectorLoad::EndeffectorLoad ()
{
  // TODO Auto-generated constructor stub
}

EndeffectorLoad::~EndeffectorLoad ()
{
  // TODO Auto-generated destructor stub
}

void
EndeffectorLoad::Init (const EndeffectorsMotion ee_motion, double dt, double T)
{
  n_contacts_per_node_.clear();
  dt_ = dt;
  double t = 0.0;
  int n = 0; // total number of discrete contacts
  for (int i=0; i<=GetNode(T); ++i) {
    int n_contacts_node = ee_motion.GetContacts(t).size();
    n_contacts_per_node_.push_back(n_contacts_node);
    n += n_contacts_node;
    t += dt;
  }

  lambdas_ = VectorXd::Zero(n);
}

void
EndeffectorLoad::SetOptimizationVariables (const VectorXd& x)
{
  lambdas_ = x;
}

EndeffectorLoad::VectorXd
EndeffectorLoad::GetOptimizationVariables () const
{
  return lambdas_;
}

int
EndeffectorLoad::GetOptVarCount () const
{
  return GetOptimizationVariables().rows();
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValues (double t) const
{
  int k = GetNode(t);
  return GetLoadValuesIdx(k);
}

EndeffectorLoad::LoadParams
EndeffectorLoad::GetLoadValuesIdx (int k_curr) const
{
  LoadParams lambda_k;
  for (int c=0; c<n_contacts_per_node_.at(k_curr); ++c)
    lambda_k.push_back(lambdas_(IndexDiscrete(k_curr,c)));

  return lambda_k;
}

int
EndeffectorLoad::GetNumberOfNodes () const
{
  return GetContactsPerNode().size();
}

int
EndeffectorLoad::Index (double t, int contact) const
{
  int k = GetNode(t);
  return IndexDiscrete(k, contact);
}

int
EndeffectorLoad::IndexDiscrete (int k_curr, int contact) const
{
  int idx = 0;
  for (int k=0; k<k_curr; ++k)
    idx += n_contacts_per_node_.at(k);

  return idx + contact;
}

int
EndeffectorLoad::GetNumberOfContacts (int k) const
{
  return GetLoadValuesIdx(k).size();
}

std::vector<int>
EndeffectorLoad::GetContactsPerNode () const
{
  return n_contacts_per_node_;
}

int
EndeffectorLoad::GetNode (double t) const
{
  return floor(t/dt_);
}

} /* namespace opt */
} /* namespace xpp */
