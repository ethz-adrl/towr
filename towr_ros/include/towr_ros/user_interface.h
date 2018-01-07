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

#ifndef XPP_VIS_USER_INTERFACE_H_
#define XPP_VIS_USER_INTERFACE_H_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Vector3.h>
#include <keyboard/Key.h>

#include <xpp_states/state.h>


namespace xpp {

/**
 * @brief Translates user input into a ROS message.
 *
 * This includes high level input about where to go (e.g. converting
 * keyboard input into a goal state), which terrain to visualize, etc.
 *
 * See the CallbackKeyboard function for the Key->Action mappings.
 */
class UserInterface {
public:

  /**
   * @brief  Constructs default object to interact with framework.
   */
  UserInterface ();
  virtual ~UserInterface () {};

private:
  ::ros::Subscriber key_sub_;          ///< the input key hits to the node.
  ::ros::Publisher  user_command_pub_; ///< the output message of the node.

  void CallbackKeyboard(const keyboard::Key& msg);
  void PublishCommand();

  State3dEuler goal_geom_;
  int kMaxNumGaits_ = 8;
  int terrain_id_;
  int gait_combo_id_;
  bool replay_trajectory_ = false;
  bool use_solver_snopt_ = false;
  bool optimize_ = false;
  bool publish_optimized_trajectory_ = false;
  double total_duration_ = 2.0;

  int AdvanceCircularBuffer(int& curr, int max) const;
};

} /* namespace xpp */

#endif /* XPP_VIS_USER_INTERFACE_H_ */
