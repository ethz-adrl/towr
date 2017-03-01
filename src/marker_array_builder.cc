/*
 * zmp_publisher.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#include <xpp/ros/marker_array_builder.h>
#include <xpp/opt/zero_moment_point.h>
#include <xpp/hyq/ee_hyq.h>

namespace xpp {
namespace ros {

static const std::string supp_tr_topic = "support_polygons";

MarkerArrayBuilder::MarkerArrayBuilder()
{
}

void MarkerArrayBuilder::AddStartStance(visualization_msgs::MarkerArray& msg,
                                        const VecFoothold& start_stance) const
{
  AddFootholds(msg, start_stance, "start_stance", visualization_msgs::Marker::CUBE, 1.0);
}

void MarkerArrayBuilder::AddSupportPolygons(visualization_msgs::MarkerArray& msg,
                                      const MotionStructure& motion_structure,
                                      const VecFootholdEig& footholds) const
{
  int phase_id = 0;
  int phase_min = 4;
  int phase_max = 5;
  for (auto phase : motion_structure.GetPhases()) {
    bool phase_in_range = phase_min<=phase_id && phase_id <=phase_max;
    if (phase.IsStep() && phase_in_range) {
      EEID swingleg = phase.swinglegs_.front().ee;
      BuildSupportPolygon(msg, phase.GetAllContacts(footholds), swingleg);
    }
    phase_id++;
  }


  // delete the other markers, maximum of 30 support polygons.
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (uint j=motion_structure.GetPhases().size(); j<30; ++j) {
    visualization_msgs::Marker marker;
    marker.id = i++;
    marker.ns = supp_tr_topic;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }

}

void
MarkerArrayBuilder::BuildSupportPolygon(
    visualization_msgs::MarkerArray& msg,
    const VecFoothold& stance,
    EEID leg_id) const
{
//  static int i=0;
//  geometry_msgs::PolygonStamped polygon_msg;
//  polygon_msg.header.frame_id = "world";
//  polygon_msg.header.seq = i++;

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker marker;
  marker.id = i;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time();
  marker.ns = supp_tr_topic; //"leg " + std::to_string(leg_id);
  marker.action = visualization_msgs::Marker::MODIFY;
 //    marker.lifetime = ros::Duration(10);
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color = GetLegColor(leg_id);
  marker.color.a = 0.15;

  static const int points_per_triangle =3;
  if (stance.size() == points_per_triangle) {
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    for (size_t i=0; i<points_per_triangle; ++i) {
      geometry_msgs::Point point;
      point.x = stance.at(i).p.x();
      point.y = stance.at(i).p.y();
      point.z = stance.at(i).p.z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // if only two contact points exist, draw line
  static const int points_per_line = 2;
  if (stance.size() == points_per_line) {
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.02;

    for (size_t i=0; i<points_per_line; ++i) {
      geometry_msgs::Point point;
      point.x = stance.at(i).p.x();
      point.y = stance.at(i).p.y();
      point.z = stance.at(i).p.z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }
}

void MarkerArrayBuilder::AddPoint(
    visualization_msgs::MarkerArray& msg,
    const Eigen::Vector2d& goal,
    std::string rviz_namespace,
    int marker_type)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  Marker marker;
  marker.id = i;
  marker = GenerateMarker(goal, marker_type, 0.02);
  marker.ns = rviz_namespace;
  marker.scale.z = 0.04;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  msg.markers.push_back(marker);
}

visualization_msgs::Marker
MarkerArrayBuilder::GenerateMarker(Eigen::Vector2d pos, int32_t type, double size) const
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = pos.x();
  marker.pose.position.y = pos.y();
  marker.pose.position.z = 0.0;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time();
  marker.type = type;
  marker.action = visualization_msgs::Marker::MODIFY;
//  marker.lifetime = ::ros::Duration(0.01);
  marker.scale.x = marker.scale.y = marker.scale.z = size;
  marker.color.a = 1.0;

  return marker;
}

void
MarkerArrayBuilder::AddLineStrip(visualization_msgs::MarkerArray& msg,
                           double center_x, double depth_x,
                           const std::string& rviz_namespace) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = frame_id_;
  line_strip.header.stamp = ::ros::Time::now();
  line_strip.id = i;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = rviz_namespace;
  line_strip.action = visualization_msgs::Marker::MODIFY;
  line_strip.pose.orientation.x = line_strip.pose.orientation.y = line_strip.pose.orientation.z = 0.0;
  line_strip.pose.orientation.w = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.2;
  line_strip.scale.x = depth_x;
  geometry_msgs::Point p1, p2;
  p1.x = center_x;
  p1.y = -0.5;
  p1.z = 0;

  p2 = p1;
  p2.y = -p1.y;

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  msg.markers.push_back(line_strip);
}

void
MarkerArrayBuilder::AddEllipse(visualization_msgs::MarkerArray& msg,
                               double center_x, double center_y,
                               double width_x, double width_y,
                               const std::string& rviz_namespace) const
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker ellipse;
  ellipse.header.frame_id = frame_id_;
  ellipse.header.stamp = ::ros::Time::now();
  ellipse.id = i;
  ellipse.type = visualization_msgs::Marker::CYLINDER;
  ellipse.ns = rviz_namespace;
  ellipse.action = visualization_msgs::Marker::MODIFY;
  ellipse.pose.position.x = center_x;
  ellipse.pose.position.y = center_y;
  ellipse.pose.orientation.x = ellipse.pose.orientation.y = ellipse.pose.orientation.z = 0.0;
  ellipse.pose.orientation.w = 1.0;
  ellipse.color.b = 1.0;
  ellipse.color.a = 0.2;

  ellipse.scale.x = width_x;
  ellipse.scale.y = width_y;
  ellipse.scale.z = 0.01; // height of cylinder

  msg.markers.push_back(ellipse);
}

void
MarkerArrayBuilder::AddBodyTrajectory(visualization_msgs::MarkerArray& msg,
                            const ComMotion& com_motion,
                            const PosXYZ offset_geom_to_com,
                            const MotionStructure& motion_structure,
                            const std::string& rviz_namespace,
                            double alpha) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

//  for (const auto& node : motion_structure.GetPhaseStampedVec())
//  {
  double dt = 0.01;  // for RA-L plots 0.002
  for (double t(0.0); t < com_motion.GetTotalTime(); t+= dt)
  {
    auto com_state = com_motion.GetCom(t);
    Vector2d body_state = com_state.p - offset_geom_to_com.segment<2>(0);

    visualization_msgs::Marker marker;
    marker = GenerateMarker(body_state,
                            visualization_msgs::Marker::SPHERE,
                            0.011);
    marker.id = i++;
    marker.ns = rviz_namespace;

    auto phase = motion_structure.GetCurrentPhase(t);
    if ( !phase.IsStep() ) {
      marker.color.r = marker.color.g = marker.color.b = 0.5;
    } else {
      auto swing_leg = phase.swinglegs_.front().ee;
      marker.color = GetLegColor(swing_leg);
    }

    marker.color.a = alpha;
    msg.markers.push_back(marker);
  }

  // delete the other markers
  for (double t=com_motion.GetTotalTime(); t < 10.0; t+= dt)
  {
    visualization_msgs::Marker marker;

    marker.id = i++;
    marker.ns = rviz_namespace;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

void
MarkerArrayBuilder::AddZmpTrajectory(visualization_msgs::MarkerArray& msg,
                                     const ComMotion& com_motion,
                                     const MotionStructure& motion_structure,
                                     double walking_height,
                                     const std::string& rviz_namespace) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;


//  for (const auto& node : motion_structure.GetPhaseStampedVec())
//  {
  double dt = 0.23;
  for (double t(0.0); t < com_motion.GetTotalTime(); t+= dt)
  {
    auto com = com_motion.GetCom(t);

    Eigen::Vector2d zmp = xpp::opt::ZeroMomentPoint::CalcZmp(com.Make3D(), walking_height);
    auto phase = motion_structure.GetCurrentPhase(t);

    visualization_msgs::Marker marker;
    marker = GenerateMarker(zmp,
                            visualization_msgs::Marker::SPHERE,
                            0.011);

    marker.ns = rviz_namespace;
    marker.id = i++;

    if ( !phase.IsStep() ) {
      marker.color.r = marker.color.g = marker.color.b = 0.5;
      marker.color.a = 0.2;
    } else {
      // take color of first swingleg
      auto swing_leg = phase.swinglegs_.front().ee;
//      marker.ns = "leg " + std::to_string(swing_leg);
//      marker.color = GetLegColor(swing_leg);
      marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
      marker.color.a = 1.0;
    }

    msg.markers.push_back(marker);
  }

  // delete the other markers
  for (double t=com_motion.GetTotalTime(); t < 10.0; t+= dt)
  {
    visualization_msgs::Marker marker;

    marker.id = i++;
    marker.ns = rviz_namespace;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

void
MarkerArrayBuilder::AddPendulum(visualization_msgs::MarkerArray& msg,
                                const ComMotion& com_motion,
                                const MotionStructure& motion_structure,
                                double walking_height,
                                const std::string& rviz_namespace,
                                double alpha) const
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time::now();
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.01; // thinkness of pendulum pole
  marker.color.a = alpha;

  // everything sent here will be overwritten
  marker.ns = rviz_namespace;
  marker.id = 0;

  double dt = 0.01;
  for (double t(0.0); t < com_motion.GetTotalTime(); t+= dt)
  {
    marker.points.clear();
    xpp::utils::StateLin2d cog_state = com_motion.GetCom(t);
    geometry_msgs::Point point;
    point.x = cog_state.p.x();
    point.y = cog_state.p.y();
    point.z = walking_height;
    marker.points.push_back(point);

    Eigen::Vector2d zmp = xpp::opt::ZeroMomentPoint::CalcZmp(cog_state.Make3D(), walking_height);
    point.x = zmp.x();
    point.y = zmp.y();
    point.z = 0.0;
    marker.points.push_back(point);


    auto phase = motion_structure.GetCurrentPhase(t);
    if ( !phase.IsStep() ) {
      marker.color.r = marker.color.g = marker.color.b = 0.1;
    } else {
      // take color of first swingleg
      auto swing_leg = phase.swinglegs_.front().ee;
      marker.color = GetLegColor(swing_leg);

    }

    msg.markers.push_back(marker);
  }

//  // delete the other markers
//  for (double t=com_motion.GetTotalTime(); t < 10.0; t+= dt)
//  {
//    visualization_msgs::Marker marker;
//
//    marker.id = i++;
//    marker.ns = rviz_namespace;
//    marker.action = visualization_msgs::Marker::DELETE;
//    msg.markers.push_back(marker);
//  }
}

void MarkerArrayBuilder::AddFootholds(
    visualization_msgs::MarkerArray& msg,
    const VecFoothold& H_footholds,
    const std::string& rviz_namespace,
    int32_t type,
    double alpha) const
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (uint j=0; j<H_footholds.size(); ++j) {

    ::geometry_msgs::Point f_curr;
    f_curr.x = H_footholds.at(j).p.x();
    f_curr.y = H_footholds.at(j).p.y();
    f_curr.z = H_footholds.at(j).p.z();

//    ::geometry_msgs::Point f_next1;
//    f_next1.x = H_footholds.at(j+1).p.x();
//    f_next1.y = H_footholds.at(j+1).p.y();
//    f_next1.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next2;
//    f_next2.x = H_footholds.at(j+1).p.x();
//    f_next2.y = H_footholds.at(j+1).p.y();
//    f_next2.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next3;
//    f_next3.x = H_footholds.at(j+1).p.x();
//    f_next3.y = H_footholds.at(j+1).p.y();
//    f_next3.z = H_footholds.at(j+1).p.z();


    // publish to rviz
    visualization_msgs::Marker marker_msg;
    marker_msg.type = type;
    marker_msg.action = visualization_msgs::Marker::MODIFY;

    marker_msg.pose.position = f_curr;
//    marker_msg.points.push_back(f_next1);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next2);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next3);


    marker_msg.header.frame_id = frame_id_;  // name of the tf message that defines the location of the body with respect to the april tag
    marker_msg.header.stamp = ::ros::Time();
    marker_msg.ns = rviz_namespace;
    marker_msg.id = i++;
//    marker_msg.lifetime = ros::Duration(10);
    marker_msg.scale.x = 0.04;
    marker_msg.scale.y = 0.04;
    marker_msg.scale.z = 0.04;


    marker_msg.color = GetLegColor(H_footholds.at(j).ee);
    marker_msg.color.a = alpha;

    msg.markers.push_back(marker_msg);
  }

  // maximum of 30 steps
  for (int k=H_footholds.size(); k < 30; k++)
  {
    visualization_msgs::Marker marker;

    marker.id = i++;
    marker.ns = rviz_namespace;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

std_msgs::ColorRGBA MarkerArrayBuilder::GetLegColor(EEID ee) const
{
  // define a few colors
  std_msgs::ColorRGBA red, green, blue, white, brown, yellow, purple;
  red.a = green.a = blue.a = white.a = brown.a = yellow.a = purple.a = 1.0;

  red.r = 1.0; red.g = 0.0; red.b = 0.0;
  green.r = 0.0; green.g = 150./255; green.b = 76./255;
  blue.r = 0.0; blue.g = 102./255; blue.b = 204./255;
  brown.r = 122./255; brown.g = 61./255; brown.b = 0.0;
  white.b = white.g = white.r = 1.0;
  yellow.r = 204./255; yellow.g = 204./255; yellow.b = 0.0;
//  purple.r = 123./255; purple.g = 104./255; purple.b = 238./255;
  purple.r = 72./255; purple.g = 61./255; purple.b = 139./255;


  std_msgs::ColorRGBA color_leg;
  using namespace xpp::hyq;
  switch (kMapOptToHyq.at(ee)) {
    case LF:
      color_leg = purple;
      break;
    case RF:
      color_leg = green;
      break;
    case LH:
      color_leg = blue;
      break;
    case RH:
      color_leg = brown;
      break;
    default:
      break;
  }

  return color_leg;
}


} /* namespace ros */
} /* namespace xpp */
