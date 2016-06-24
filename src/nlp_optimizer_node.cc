/*
 * xpp_optimizer_node.cc
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#include <xpp/ros/nlp_optimizer_node.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp/zmp/optimization_variables_interpreter.h>

// for calculating if initial stance phase neccessary, fixme move to own class
#include <xpp/zmp/zero_moment_point.h>
#include <xpp/zmp/zmp_constraint_builder.h>

namespace xpp {
namespace ros {

NlpOptimizerNode::NlpOptimizerNode ()
{
  current_info_sub_ = n_.subscribe("required_info_nlp",
                                   1, // take only the most recent information
                                   &NlpOptimizerNode::CurrentInfoCallback, this);

  opt_params_pub_ = n_.advertise<OptParamMsg>("optimized_parameters_nlp", 1);

  // get current optimization values from the optimizer
  optimization_visualizer_.SetObserver(nlp_facade_.GetObserver());
  nlp_facade_.AttachVisualizer(optimization_visualizer_);
  prev_swingleg_ = hyq::RF;
}

void
NlpOptimizerNode::CurrentInfoCallback(const ReqInfoMsg& msg)
{
  // fixme DRY: use template method to move this and qp code to base class
  UpdateCurrentState(msg);
  OptimizeTrajectory();
  PublishOptimizedValues();
}

void
NlpOptimizerNode::UpdateCurrentState(const ReqInfoMsg& msg)
{
  curr_cog_      = RosHelpers::RosToXpp(msg.curr_state);
  curr_stance_   = RosHelpers::RosToXpp(msg.curr_stance);
  step_sequence_ = DetermineStepSequence(curr_cog_, msg.curr_swingleg);
}

void
NlpOptimizerNode::PublishOptimizedValues() const
{
  OptParamMsg msg_out;
  msg_out.splines   = xpp::ros::RosHelpers::XppToRos(opt_splines_);
  msg_out.footholds = xpp::ros::RosHelpers::XppToRos(footholds_);

  opt_params_pub_.publish(msg_out);
}

void
NlpOptimizerNode::OptimizeTrajectory()
{

  xpp::zmp::ContinuousSplineContainer spline_structure;
  bool add_final_stance = true;
  spline_structure.Init(curr_cog_.Get2D().p, curr_cog_.Get2D().v, step_sequence_.size(),
                        spline_times_, start_with_com_shift_, add_final_stance);
  spline_structure.SetEndAtStart();

  auto interpreter_ptr = std::make_shared<xpp::zmp::OptimizationVariablesInterpreter>();
  interpreter_ptr->Init(spline_structure, step_sequence_, curr_stance_, robot_height_);

  nlp_facade_.SolveNlp(curr_cog_.Get2D().a,
                       goal_cog_.Get2D(),
                       interpreter_ptr);

  opt_splines_ = nlp_facade_.GetSplines();
  footholds_   = nlp_facade_.GetFootholds();
}

std::vector<xpp::hyq::LegID>
NlpOptimizerNode::DetermineStepSequence(const State& curr_state, int curr_swingleg)
{
  // TODO make step sequence dependent on curr_state
  const double length_per_step = 0.30;
  const double width_per_step = 0.20;
  Eigen::Vector2d start_to_goal = goal_cog_.p.segment<2>(0) - curr_cog_.p.segment<2>(0);


  int req_steps_per_leg;
  // fixme don't take steps if body movement is sufficient
  if (false /*start_to_goal.norm() < 0.1*/) {
    req_steps_per_leg = 0;
  } else {
    int req_steps_by_length = std::ceil(std::fabs(start_to_goal.x())/length_per_step);
    int req_steps_by_width  = std::ceil(std::fabs(start_to_goal.y())/width_per_step);
    req_steps_per_leg = std::max(req_steps_by_length,req_steps_by_width);
  }



  // don't do anything if goal to close
  if (start_to_goal.norm() < 0.05) {
    std::cout << "goal closer than 0.05m\n";
    start_with_com_shift_ = false;
    return std::vector<xpp::hyq::LegID>(); // empty vector, take no steps
  }


  LegID sl;
  if (curr_swingleg == hyq::NO_SWING_LEG)
    sl = prev_swingleg_;
  else {
    sl = static_cast<LegID>(curr_swingleg);
    prev_swingleg_ = sl;
  }




  using namespace xpp::hyq;
  std::vector<xpp::hyq::LegID> step_sequence;


  for (int step=0; step<req_steps_per_leg*4; ++step) {
    step_sequence.push_back(NextSwingLeg(sl));
    sl = step_sequence.back();
  }




  // determine, whether initial stance phase must be inserted
  Eigen::Vector2d zmp = xpp::zmp::ZeroMomentPoint::CalcZmp(curr_state, robot_height_);

  // remove first swingleg from current stance
  VecFoothold first_stance = curr_stance_;
  LegID first_swingleg = step_sequence.front();
  int idx_swingleg = Foothold::GetLastIndex(first_swingleg, first_stance);
  first_stance.erase(first_stance.begin() + idx_swingleg);

  // fixme zero margins, since i actually allow violation of zmp constraint
  // don't want 4ls to be inserted there
  MarginValues margins = hyq::SupportPolygon::GetDefaultMargins();
  margins.at(DIAG)/=2.;
  hyq::SupportPolygon supp(first_stance, margins);

  bool zmp_inside = zmp::ZmpConstraintBuilder::IsZmpInsideSuppPolygon(zmp,supp);

  start_with_com_shift_ = false;
  // so 4ls-phase not always inserted b/c of short time zmp constraints are ignore
  // when switching between disjoint support triangles.
  if (!zmp_inside /*&& curr_state.v.norm() < 0.01*/)
    start_with_com_shift_ = true;

  return step_sequence;
}

NlpOptimizerNode::LegID
NlpOptimizerNode::NextSwingLeg(LegID curr) const
{
  using namespace xpp::hyq;

  switch (curr) {
    case LH: return LF;
    case LF: return RH;
    case RH: return RF;
    case RF: return LH;
    default: assert(false); // this should never happen
  };
}

} /* namespace ros */
} /* namespace xpp */
