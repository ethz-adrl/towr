/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/spline.h>

#include <cassert>

#include <xpp/opt/variables/variable_names.h>

#include <xpp/opt/variables/node_values.h>
#include <xpp/opt/variables/coeff_spline.h>

namespace xpp {
namespace opt {

Spline::Ptr
Spline::BuildSpline (const OptVarsPtr& opt_vars,
                     const std::string& name,
                     const VecTimes& poly_durations)
{
  Ptr spline;

  std::string s1 = id::endeffectors_motion;
  std::string s2 = id::endeffector_force;
  if (name.substr(0, s1.size()) == s1) // string starts with s
    spline = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(name));
  else if (name.substr(0, s2.size()) == s2) // string starts with s
    spline = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(name));
  else if (name == id::base_linear)
    spline = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(name));
//    spline = std::make_shared<CoeffSpline>(opt_vars, name, poly_durations);
  else if (name == id::base_angular)
    spline = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(name));
//    spline = std::make_shared<CoeffSpline>(opt_vars, name, poly_durations);
  else
    assert(false); // this shouldn't happen

  return spline;
}


Spline::Spline ()
{
}

Spline::~Spline ()
{
}

int
Spline::GetSegmentID(double t_global, const VecTimes& durations)
{
  double eps = 1e-10; // double imprecision
  assert(t_global >= 0.0);

   double t = 0;
   int i=0;
   for (double d: durations) {
     t += d;

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }

   assert(false); // this should never be reached
}

Spline::LocalInfo
Spline::GetLocalTime (double t_global, const VecTimes& durations)
{
  int id = GetSegmentID(t_global, durations);

  double t_local = t_global;
  for (int i=0; i<id; i++)
    t_local -= durations.at(i);

  return std::make_pair(id, t_local);
}


} /* namespace opt */
} /* namespace xpp */

