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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <xpp_states/convert.h>

#include <towr_ros/TowrCommand.h>  // listen to goal state
#include <towr_ros/topic_names.h>

static ros::Publisher rviz_pose_pub;

void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
{
  // publish goal pose
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.pose.position = msg_in.goal_lin.pos;
  Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(msg_in.goal_ang.pos.z,
                                                        msg_in.goal_ang.pos.y,
                                                        msg_in.goal_ang.pos.x);
  goal_msg.pose.orientation = xpp::Convert::ToRos(q);
  rviz_pose_pub.publish(goal_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "goal_pose_publisher");

  ros::NodeHandle n;

  ros::Subscriber goal_sub;
  goal_sub      = n.subscribe(towr_msgs::user_command, 1, UserCommandCallback);
  rviz_pose_pub = n.advertise<geometry_msgs::PoseStamped>("xpp/goal", 1);

  ros::spin();

  return 1;
}
