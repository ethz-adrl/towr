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

#include <towr_ros/TowrCommand.h> // listen to goal state
#include <towr_ros/topic_names.h>
#include <towr_ros/rviz_terrain_builder.h>

static ros::Publisher rviz_terrain_pub;
static towr::RvizTerrainBuilder terrain_builder;

void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
{
  // draw a terrain
  auto msg = terrain_builder.BuildTerrain(msg_in.terrain_id);
  rviz_terrain_pub.publish(msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rviz_terrain_visualizer");

  ros::NodeHandle n;

  ros::Subscriber goal_sub;
  goal_sub         = n.subscribe(towr_msgs::user_command, 1, UserCommandCallback);
  rviz_terrain_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/terrain", 1);

  ros::spin();

  return 1;
}
