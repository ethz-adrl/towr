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
  double t = 0.0;
  int n = 0; // total number of discrete contacts
  for (int i=0; i<T/dt; ++i) {
    int n_contacts_node = ee_motion.GetContacts(t).size();
    n_contacts_per_node_.push_back(n_contacts_node);
    n += n_contacts_node;
    t += dt;
  }

  lambdas_ = VectorXd(n);
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

std::vector<int>
EndeffectorLoad::GetContactsPerNode () const
{
  return n_contacts_per_node_;
}

} /* namespace opt */
} /* namespace xpp */
