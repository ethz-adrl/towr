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
  service_ = n.advertiseService("optimize_trajectory", &XppOptimizerNode::OptimizeTrajectoryService, this);

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


bool
XppOptimizerNode::OptimizeTrajectoryService(xpp_opt::OptimizeTrajectory::Request& req,
                                            xpp_opt::OptimizeTrajectory::Response& res)
{
  goal_cog_ = StateLinMsgTo2DState(req.goal_state);

  Eigen::VectorXd opt_coefficients;
  StdVecEigen2d opt_footholds;

  OptimizeTrajectory(opt_coefficients, opt_footholds);


  //FIXME don't use for loop, copy data directly
  for (int i=0; i<opt_coefficients.rows(); ++i)
    res.x.spline_coeff.push_back(opt_coefficients[i]);
  for (int i=0; i<opt_footholds.size(); ++i) {
    geometry_msgs::Point p;
    p.x = opt_footholds.at(i).x();
    p.y = opt_footholds.at(i).y();
    res.x.footholds.push_back(p);
    std::cout << "p:" << p;
  }

  std::cout << "\n\n " << res.x.footholds.size() << "\n";
  return true;
}


void
XppOptimizerNode::OptimizeTrajectory(VectorXd& opt_coefficients,
                                     StdVecEigen2d& opt_footholds) const
{
  std::vector<xpp::hyq::LegID> step_sequence = DetermineStepSequence();

  nlp_optimizer_.SolveNlp(curr_cog_,
                          goal_cog_,
                          step_sequence,
                          curr_stance_,
                          opt_coefficients,
                          opt_footholds);
}


std::vector<xpp::hyq::LegID>
XppOptimizerNode::DetermineStepSequence() const
{
  const double length_per_step = 0.25;
  const double width_per_step = 0.15;
  Eigen::Vector2d start_to_goal = goal_cog_.p.segment<2>(0) - curr_cog_.p.segment<2>(0);

  int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/length_per_step);
  int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
  // get greatest value of all
  int req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);

  using namespace xpp::hyq;
  const std::vector<xpp::hyq::LegID> take_4_steps = {LH, LF, RH, RF};
  std::vector<xpp::hyq::LegID> step_sequence;
  for (int i=0; i<req_steps_per_leg; ++i) {
    // insert 4 steps
    step_sequence.insert(step_sequence.end(), take_4_steps.begin(), take_4_steps.end());
  }

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
