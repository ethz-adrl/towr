/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/variables/spline.h>

#include <xpp/variables/node_values.h>
#include <xpp/variables/variable_names.h>


namespace xpp {


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


} /* namespace xpp */

