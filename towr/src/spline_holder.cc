/*
 * spline_holder.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: winklera
 */

#include <towr/variables/spline_holder.h>

namespace towr
{

SplineHolder::SplineHolder ()
{
  // TODO Auto-generated constructor stub
}

SplineHolder::SplineHolder (NodeValues::Ptr base_lin_nodes,
                            NodeValues::Ptr base_ang_nodes,
                            const std::vector<double>& base_poly_durations,
                            std::vector<NodeValues::Ptr> ee_motion_nodes,
                            std::vector<NodeValues::Ptr> ee_force_nodes,
                            std::vector<ContactSchedule::Ptr> contact_schedule)
{

  base_linear_ = std::make_shared<Spline>(base_lin_nodes, base_poly_durations);
  base_lin_nodes->AddObserver(base_linear_);

  base_angular_ = std::make_shared<Spline>(base_ang_nodes, base_poly_durations);
  base_ang_nodes->AddObserver(base_angular_);


  for (uint ee=0; ee<ee_motion_nodes.size(); ++ee) {

    auto ee_spline = std::make_shared<Spline>(ee_motion_nodes.at(ee), contact_schedule.at(ee)->GetDurations());
    ee_motion_.push_back(ee_spline);
    ee_motion_nodes.at(ee)->AddObserver(ee_spline);

    auto ee_force_spline = std::make_shared<Spline>(ee_force_nodes.at(ee), contact_schedule.at(ee)->GetDurations());
    ee_force_.push_back(ee_force_spline);
    ee_force_nodes.at(ee)->AddObserver(ee_force_spline);

    // should link these nonetheless
//    if (optimize_timings_) {

    ContactSchedule::Ptr c = contact_schedule.at(ee);

      // smell dependes on order (must be this way)
      ee_motion_.at(ee)->SetContactSchedule(c);
      c->AddObserver(ee_motion_.at(ee));

      ee_force_.at(ee)->SetContactSchedule(c);
      c->AddObserver(ee_force_.at(ee));
//    }

  }

}

} /* namespace towr */
