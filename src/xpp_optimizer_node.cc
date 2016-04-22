/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/xpp_optimizer_node.h>
#include <xpp_opt/OptimizedVariables.h>

namespace xpp {
namespace ros {

XppOptimizerNode::XppOptimizerNode ()
{
  // TODO Auto-generated constructor stub
  ::ros::NodeHandle n;
  opt_var_pub_ = n.advertise<xpp_opt::OptimizedVariables>("opt_variables",10);
  curr_state_sub_ = n.subscribe("curr_state", 10, &XppOptimizerNode::CurrentStateCallback, this);
  goal_state_sub_ = n.subscribe("goal_state", 10, &XppOptimizerNode::GoalStateCallback, this);

  //fixme get this from robot
  using namespace xpp::hyq;
  curr_stance_[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  curr_stance_[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  curr_stance_[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  curr_stance_[RH] = Foothold(-0.35, -0.3, 0.0, RH);

}

XppOptimizerNode::~XppOptimizerNode ()
{
  // TODO Auto-generated destructor stub
}

//FIXME make this a ros service maybe?
void
XppOptimizerNode::OptimizeTrajectory() const
{
  StdVecEigen2d opt_footholds_2d;

  std::vector<xpp::hyq::LegID> step_sequence = DetermineStepSequence();



  Eigen::VectorXd opt_coefficients = nlp_optimizer_.SolveNlp(curr_cog_,
                                                            goal_cog_,
                                                            step_sequence,
                                                            curr_stance_,
                                                            opt_footholds_2d);

  //FIXME don't use for loop, copy data directly
  xpp_opt::OptimizedVariables msg;
  for (int i=0; i<opt_coefficients.rows(); ++i)
    msg.spline_coeff.push_back(opt_coefficients[i]);
  for (int i=0; i<opt_footholds_2d.size(); ++i) {
    geometry_msgs::Point p;
    p.x = opt_footholds_2d.at(i).x();
    p.y = opt_footholds_2d.at(i).y();
    msg.footholds.push_back(p);
  }

  opt_var_pub_.publish(msg);
}

std::vector<xpp::hyq::LegID>
XppOptimizerNode::DetermineStepSequence() const
{
  using namespace xpp::hyq;
  // FIXME something start - goal and watch order
  std::vector<xpp::hyq::LegID> step_sequence = {LH, LF, RH, RF, LH, LF, RH, RF};
  return step_sequence;
}


void
XppOptimizerNode::CurrentStateCallback(const xpp_opt::StateLin3d& msg)
{
  curr_cog_ = StateLinMsgTo2DState(msg);
}


void
XppOptimizerNode::GoalStateCallback(const xpp_opt::StateLin3d& msg)
{
  goal_cog_ = StateLinMsgTo2DState(msg);
  OptimizeTrajectory();
}


XppOptimizerNode::State
XppOptimizerNode::StateLinMsgTo2DState(const xpp_opt::StateLin3d& msg) const
{
  State point;
  point.p.x() = msg.pos.x;
  point.p.y() = msg.pos.y;

  point.v.x() = msg.vel.x;
  point.v.y() = msg.vel.y;

  point.a.x() = msg.acc.x;
  point.a.y() = msg.acc.y;

  return point;
}



} /* namespace ros */
} /* namespace xpp */
