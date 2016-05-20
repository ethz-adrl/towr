/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/zmp/nlp_optimizer.h>
#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace zmp {

class NlpOptimizerTest : public  ::testing::TestWithParam< double > {
public:
  typedef xpp::utils::Point2d Point2d;
  typedef xpp::utils::Point3d Point3d;
  typedef NlpOptimizer::VecSpline VecSpline;
  typedef NlpOptimizer::VecFoothold VecFoothold;

protected:
  virtual void SetUp()
  {
    using namespace xpp::hyq;

    NlpOptimizer nlp_optimizer;
    start_xy_.p << 0.01, 0.4,  0.7;
    start_xy_.v << 0.02, 0.5,  0.8;
    start_xy_.a << 0.03, 0.6,  0.9;

    goal_xy_.p <<  0.10, 0.11;
    goal_xy_.v <<  0.12, 0.13;
    goal_xy_.a <<  0.14, 0.15;

    start_stance_.push_back(Foothold(-0.31,  0.37, 0.0, LH));
    start_stance_.push_back(Foothold( 0.33,  0.35, 0.0, LF));
    start_stance_.push_back(Foothold(-0.35, -0.33, 0.0, RH));
    start_stance_.push_back(Foothold( 0.37, -0.31, 0.0, RF));


    times_.t_stance_ = 0.2;
    times_.t_swing_ = 0.7;
    times_.t_stance_initial_ = 0.5; // this will create two initial splines
    times_.t_stance_final_ = 0.2;

    nlp_optimizer.SolveNlp(start_xy_, goal_xy_,
                           {LH, LF, RH, RF}, start_stance_,
                           times_, robot_height_,
                           opt_xy_splines_, opt_footholds_);
  }

  static constexpr double robot_height_ = 0.58;
  SplineTimes times_;
  Point2d start_xy_, goal_xy_;
  VecFoothold start_stance_;

  VecSpline opt_xy_splines_;
  VecFoothold opt_footholds_;
};


} // namespace zmp
} // namespace xpp
