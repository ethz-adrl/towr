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
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <towr_ros/TowrCommand.h>
#include <towr_ros/topic_names.h>
#include <towr/terrain/height_map.h>


namespace towr {

static ros::Publisher rviz_pub;

void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
{
  // get which terrain
  auto terrain_id = static_cast<HeightMap::TerrainID>(msg_in.terrain);
  auto terrain_ = HeightMap::MakeTerrain(terrain_id);

  // x-y area patch that should be drawn in rviz
  double dxy   =  0.06;
  double x_min = -1.0;
  double x_max =  4.0;
  double y_min = -1.0;
  double y_max =  1.0;

  visualization_msgs::Marker m;
  int id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.scale.z = 0.003;
  m.ns = "terrain";
  m.header.frame_id = "world";
  m.color.r = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.a = 0.65;

  visualization_msgs::MarkerArray msg;
  double x =  x_min;
  while (x < x_max) {
    double y = y_min;
    while (y < y_max) {
      // position
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = terrain_->GetHeight(x,y);

      // orientation
      Eigen::Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, x, y);
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), n);
      m.pose.orientation.w = q.w();
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();

      // enlarge surface-path when tilting
      double gain = 1.5;
      m.scale.x = (1+gain*n.cwiseAbs().x())*dxy;
      m.scale.y = (1+gain*n.cwiseAbs().y())*dxy;


      m.id = id++;
      msg.markers.push_back(m);

      y += dxy;
    }
    x += dxy;
  }

  rviz_pub.publish(msg);
}

} // namespace towr

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rviz_terrain_visualizer");

  ros::NodeHandle n;

  ros::Subscriber goal_sub;
  goal_sub       = n.subscribe(towr_msgs::user_command, 1, towr::UserCommandCallback);
  towr::rviz_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/terrain", 1);

  ros::spin();

  return 1;
}
