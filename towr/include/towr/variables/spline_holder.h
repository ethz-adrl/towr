/*
 * spline_holder.h
 *
 *  Created on: Jan 9, 2018
 *      Author: winklera
 */
/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_

#include "contact_schedule.h"
#include "node_variables.h"
#include "spline.h"


namespace towr {

/**
 * This class is responsible for holding Pointers to fully constructed
 * splines, that are linked to the optimization variables.
 *
 * This is independent from whether they are added as optimization variables
 */
class SplineHolder {
public:
  using EndeffectorID = uint;


  // smell remove this
  SplineHolder () = default;
  virtual ~SplineHolder () = default;


  SplineHolder (NodeVariables::Ptr base_lin,
                NodeVariables::Ptr base_ang,
                const std::vector<double>& base_poly_durations,
                std::vector<NodeVariables::Ptr> ee_motion,
                std::vector<NodeVariables::Ptr> ee_force,
                std::vector<ContactSchedule::Ptr> contact_schedule,
                bool ee_durations_change);


  Spline::Ptr GetBaseLinear() const { return base_linear_; };
  Spline::Ptr GetBaseAngular() const { return base_angular_; };

  std::vector<Spline::Ptr> GetEEMotion() const { return ee_motion_; };
  std::vector<Spline::Ptr> GetEEForce()  const { return ee_force_; };
  Spline::Ptr GetEEMotion(EndeffectorID ee) const { return ee_motion_.at(ee); };
  Spline::Ptr GetEEForce(EndeffectorID ee)  const { return ee_force_.at(ee); };


private:
  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;

  std::vector<Spline::Ptr> ee_motion_;
  std::vector<Spline::Ptr> ee_force_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_ */
