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

#include <towr_ros/rviz_terrain_builder.h>

#include <cmath>
#include <string>
#include <vector>

#include <xpp_states/convert.h>
#include <xpp_states/state.h>
#include <xpp_vis/rviz_colors.h>

#include <towr/height_map.h>

namespace xpp {


RvizTerrainBuilder::Marker
RvizTerrainBuilder::BuildTerrainBlock (const Vector3d& pos,
                                       const Vector3d& edge_length,
                                       const Quat& ori) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = Convert::ToRos<geometry_msgs::Point>(pos);
  m.pose.orientation = Convert::ToRos(ori);
  m.scale            = Convert::ToRos<geometry_msgs::Vector3>(edge_length);
  m.ns = "terrain";
  m.header.frame_id = rviz_frame_;
  m.color = color.wheat;

  return m;
}

geometry_msgs::PoseStamped
RvizTerrainBuilder::BuildPose (const geometry_msgs::Point pos,
                               xpp_msgs::StateLin3d orientation) const
{
  geometry_msgs::PoseStamped msg_out;
  msg_out.header.frame_id = rviz_frame_;

  // linear part
  msg_out.pose.position = pos;

  // angular part
  auto goal_ang = Convert::ToXpp(orientation);
  Eigen::Quaterniond q = GetQuaternionFromEulerZYX(goal_ang.p_.z(),
                                                   goal_ang.p_.y(),
                                                   goal_ang.p_.x());
  msg_out.pose.orientation = Convert::ToRos(q);

  return msg_out;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrain (int terrain)
{
  MarkerArray msg;

  switch (terrain) {
    case towr::FlatID:      msg = BuildTerrainFlat(); break;
    case towr::BlockID:     msg = BuildTerrainBlock(); break;
    case towr::StairsID:    msg = BuildTerrainStairs(); break;
    case towr::GapID:       msg = BuildTerrainGap(); break;
    case towr::SlopeID:     msg = BuildTerrainSlope(); break;
    case towr::ChimneyID:   msg = BuildTerrainChimney(); break;
    case towr::ChimneyLRID: msg = BuildTerrainChimneyLR(); break;
    default: return MarkerArray(); // terrain visualization not implemented
  }

  int id = terrain_ids_start_;
  for (Marker& m : msg.markers)
    m.id = id++;

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainFlat() const
{
  MarkerArray msg;

  for (int i=0; i<terrain_ids_start_; ++i) {
    msg.markers.push_back(BuildTerrainBlock(Vector3d(), Vector3d()));
    msg.markers.back().color.a = 0.0;
  }

  // one long path
  Vector3d size_start_end(5,1,0.1);
  Vector3d center0(1.5, 0.0, -0.05-eps_);
  msg.markers.at(0) = BuildTerrainBlock(center0, size_start_end);

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainBlock() const
{
  double block_start = 1.5;
  double length_     = 3.5;
  double height_     = 0.8; // [m]


  MarkerArray msg;
  double area_width = 3.0;
  double ground_thickness = 0.1;

  Vector3d size0(3,area_width,ground_thickness);
  Vector3d center0(0, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));



  Vector3d size(length_,area_width,height_+ground_thickness);
  Vector3d center1(size.x()/2 + block_start, 0.0, size.z()/2-eps_-ground_thickness);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  return msg;
}


RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainStairs() const
{
  double first_step_start = 1.5;
  double height_first_step = 0.2;
  double first_step_width = 0.4;
  double width_top = 1.0;


  MarkerArray msg;
  double area_width = 3.0;

  Vector3d size0(6.5,area_width,0.1);
  Vector3d center0(2.25, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));


  Vector3d size(first_step_width+width_top,area_width,height_first_step);
  Vector3d center1(size.x()/2 + first_step_start, 0.0, size.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  double height_second_step = 0.4;
  Vector3d size2(width_top,area_width,height_second_step);
  Vector3d pos2(first_step_start+first_step_width+size2.x()/2, 0.0, size2.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size2));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainGap() const
{
  MarkerArray msg;

  double gap_start = 1.5;
  double l_gap = 0.5;      // was 0.5m or 1m for biped  for anymal motions

  double lx = gap_start*2.0;
  double ly = 3.0;
  double lz = 2.04;


  Vector3d size0(4.5,1,0.04);
  Vector3d center0(1.25, 0.0, -lz-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));
  msg.markers.back().color.a = 0.0;

  Vector3d size(lx,ly,lz);
  Vector3d center1(0.0, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  Vector3d pos2(l_gap + lx, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainSlope() const
{
  MarkerArray msg;
  double area_width = 3.0;

  const double slope_start = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;
  const double x_down_start_ = slope_start+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope = height_center/up_length_;

  double length_start_end_ = 2.0; // [m]


  Vector3d size_start_end(2,area_width,0.04);
  Vector3d center0(-length_start_end_/2. + slope_start, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));


  double roll = 0.0;
  double pitch = -atan(slope);
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = height_center/sin(pitch);
  double ly = area_width;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  // slope up
  Vector3d center1(slope_start+up_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));

  // slope_down
  Vector3d center2(x_down_start_+down_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));

  // flat end
  Vector3d center_end(length_start_end_/2.+x_flat_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));


  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainChimney() const
{
  MarkerArray msg;
  double area_width = 3.0;

  const double x_start_ = 1.5;
  const double length_  = 1.5;
  const double y_start_ = 0.5; // for rosbag was: 0.8  or  0.5; distance to start of slope from center at z=0
  const double slope    = 3;   // for rosbag was: 2    or  3

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 4.0;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,area_width,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, -3*eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 1.0;

//  // slope_right
//  Vector3d center2(x_start_+length_/2, -y_start_-eps_, 0);
//  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
//  msg.markers.back().color.a = 0.8;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

RvizTerrainBuilder::MarkerArray
RvizTerrainBuilder::BuildTerrainChimneyLR() const
{
  MarkerArray msg;
  double area_width=3.0;

  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope    = 2;

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 2.5;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,area_width,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 1.0;

  // slope_right
  Vector3d center2(center1.x()+length_, -y_start_-eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
  msg.markers.back().color.a = 1.0;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+2*length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

} /* namespace xpp */
