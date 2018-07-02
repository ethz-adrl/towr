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
 * Mapping semantic information (e.g. name of the foot) between
 * towr and xpp domain.
 */
#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_

#include <map>
#include <towr/models/examples/endeffector_mappings.h>
#include <xpp_states/endeffector_mappings.h>


namespace towr {

static std::map<towr::QuadrupedIDs, xpp::quad::FootIDs> to_xpp_quad =
{
    {towr::LF, xpp::quad::LF},
    {towr::RF, xpp::quad::RF},
    {towr::LH, xpp::quad::LH},
    {towr::RH, xpp::quad::RH}
};

static std::map<towr::BipedIDs, xpp::biped::FootIDs> to_xpp_biped =
{
    {towr::L, xpp::biped::L},
    {towr::R, xpp::biped::R},
};

/**
 * Converts endeffector IDs of towr into the corresponding number used in xpp.
 *
 * @param number_of_ee  Number of endeffectors of current robot model.
 * @param towr_ee_id    Integer used to represent the endeffector inside towr.
 * @return corresponding endeffector in the xpp domain.
 */
static xpp::EndeffectorID ToXppEndeffector(int number_of_ee, int towr_ee_id)
{
  switch (number_of_ee) {
    case 1: return towr_ee_id;
            break;
    case 2: return to_xpp_biped.at(static_cast<towr::BipedIDs>(towr_ee_id));
            break;
    case 4: return to_xpp_quad.at(static_cast<towr::QuadrupedIDs>(towr_ee_id));
            break;
    default: assert(false); // endeffector mapping not defined
    break;
  }
}

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_TOWR_XPP_EE_MAP_H_ */
