/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/models/hyq_model.h>

#include <xpp_states/endeffector_mappings.h>

namespace towr {

HyqKinematicModel::HyqKinematicModel () : KinematicModel(4)
{
  const double x_nominal_b = 0.31;
  const double y_nominal_b = 0.29;
  const double z_nominal_b = -0.58;

  nominal_stance_.at(xpp::quad::LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  max_dev_from_nominal_ << 0.15, 0.06, 0.1;
}

} /* namespace towr */
