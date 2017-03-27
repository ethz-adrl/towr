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
  dt_ = dt;


//  n_contacts_per_node_.clear();
//  double t = 0.0;
//  int n = 0; // total number of discrete contacts
//  for (int i=0; i<=GetNode(T); ++i) {
//    int n_contacts_node = ee_motion.GetContacts(t).size();
//    n_contacts_per_node_.push_back(n_contacts_node);
//    n += n_contacts_node;
//    t += dt;
//  }
//
//  lambdas_ = VectorXd::Zero(n);


  n_ee_ = ee_motion.GetNumberOfEndeffectors();
  int num_parameters = n_ee_ * GetSegment(T);
  lambdas_new_ = VectorXd::Zero(num_parameters);

}

void
EndeffectorLoad::SetOptimizationVariables (const VectorXd& x)
{
//  lambdas_ = x;

  lambdas_new_ = x;
}

EndeffectorLoad::VectorXd
EndeffectorLoad::GetOptimizationVariables () const
{
//  return lambdas_;
  return lambdas_new_;
}

int
EndeffectorLoad::GetOptVarCount () const
{
  return GetOptimizationVariables().rows(); // spring_clean_ should be done in base class
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
//  LoadParams lambda_k;
//  for (int c=0; c<n_contacts_per_node_.at(k_curr); ++c)
//    lambda_k.push_back(lambdas_(IndexDiscrete(k_curr,c)));

  LoadParams load(n_ee_);
  for (auto ee : load.GetEEsOrdered())
    load.At(ee) = lambdas_new_(IndexDiscrete(k_curr,ee));

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
//  int idx = 0;
//  for (int k=0; k<k_curr; ++k)
//    idx += n_contacts_per_node_.at(k);

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

//int
//EndeffectorLoad::GetNumberOfContacts (int k) const
//{
//  return GetLoadValuesIdx(k).size();
//}

//std::vector<int>
//EndeffectorLoad::GetContactsPerNode () const
//{
//  return n_contacts_per_node_;
//}

int
EndeffectorLoad::GetSegment (double t) const
{
  return floor(t/dt_);
}

} /* namespace opt */
} /* namespace xpp */

