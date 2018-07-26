/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

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

/**
 * @file towr_xpp_ee_map.h
 *
 * Mapping information/types between towr and xpp domain for visualization.
 */
#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_

#include <map>
#include <towr/models/endeffector_mappings.h>
#include <xpp_states/endeffector_mappings.h>


namespace towr {

/** Mapping endeffector IDs */
static std::map<towr::BipedIDs, xpp::biped::FootIDs> biped_to_xpp_id =
{
    {L, xpp::biped::L},
    {R, xpp::biped::R},
};

static std::map<towr::QuadrupedIDs, xpp::quad::FootIDs> quad_to_xpp_id =
{
    {LF, xpp::quad::LF},
    {RF, xpp::quad::RF},
    {LH, xpp::quad::LH},
    {RH, xpp::quad::RH}
};


/** Mapping endeffector names */
static std::map<towr::BipedIDs, std::string> biped_to_name =
{
  {L, "Left" },
  {R, "Right"}
};

static std::map<towr::QuadrupedIDs, std::string> quad_to_name =
{
  {LF, "Left-Front" },
  {RF, "Right-Front"},
  {LH, "Left-Hind"  },
  {RH, "Right-Hind" }
};


/**
 * Converts endeffector IDs of towr into the corresponding number used in xpp.
 *
 * @param number_of_ee  Number of endeffectors of current robot model.
 * @param towr_ee_id    Integer used to represent the endeffector inside towr.
 * @return corresponding endeffector and string name in the xpp domain.
 */

static std::pair<xpp::EndeffectorID, std::string>
ToXppEndeffector(int number_of_ee, int towr_ee_id)
{
  std::pair<xpp::EndeffectorID, std::string> ee;

  switch (number_of_ee) {
    case 1:
      ee.first  = towr_ee_id;
      ee.second = "E0";
      break;
    case 2: {
      auto id = static_cast<towr::BipedIDs>(towr_ee_id);
      ee.first  = biped_to_xpp_id.at(id);
      ee.second = biped_to_name.at(id);
      break;
    }
    case 4: {
      auto id = static_cast<towr::QuadrupedIDs>(towr_ee_id);
      ee.first  = quad_to_xpp_id.at(id);
      ee.second = quad_to_name.at(id);
      break;
    }
    default:
      assert(false); // endeffector mapping not defined
      break;
  }

  return ee;
}

/**
 * Converts class "State" between two domains (have same internal representation).
 */
static xpp::StateLinXd ToXpp(const towr::State& towr)
{
  xpp::StateLinXd xpp(towr.p().rows());

  xpp.p_ = towr.p();
  xpp.v_ = towr.v();
  xpp.a_ = towr.a();

  return xpp;
}

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_ */
