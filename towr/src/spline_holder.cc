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
                            std::vector<ContactSchedule::Ptr> contact_schedule,
                            bool durations_change)
{

  base_linear_ = std::make_shared<Spline>(base_lin_nodes.get(), base_poly_durations);
  base_angular_ = std::make_shared<Spline>(base_ang_nodes.get(), base_poly_durations);

  for (uint ee=0; ee<ee_motion_nodes.size(); ++ee) {

    if (durations_change) {

      ee_motion_.push_back(std::make_shared<Spline>(ee_motion_nodes.at(ee).get(), contact_schedule.at(ee).get()));
      ee_force_.push_back(std::make_shared<Spline>(ee_force_nodes.at(ee).get(), contact_schedule.at(ee).get()));
    } else {
      ee_motion_.push_back(std::make_shared<Spline>(ee_motion_nodes.at(ee).get(), contact_schedule.at(ee)->GetDurations()));
      ee_force_.push_back(std::make_shared<Spline>(ee_force_nodes.at(ee).get(), contact_schedule.at(ee)->GetDurations()));
    }

  }
}

} /* namespace towr */
