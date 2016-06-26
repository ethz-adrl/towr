/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/zmp/nlp_facade.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>
#include <xpp/hyq/foothold.h>

#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace zmp {

/** Test the NlpFacade by solving an optimization problem.
  *
  * Remember to run the optimization from a folder, where a ipopt.opt config
  * files is present (xpp/test/google/).
  */
class NlpFacadeTest : public  ::testing::Test {
public:
  typedef NlpFacade::State Point2d;
  typedef NlpFacade::VecSpline VecSpline;
  typedef NlpFacade::VecFoothold VecFoothold;

protected:
  /** optimizes the spline only once for all tests that are build from this
    * test case.
    */
  static void SetUpTestCase()
  {
    using namespace xpp::hyq;

    NlpFacade nlp_facade;
    start_xy_.p << 0.01, 0.04;
    start_xy_.v << 0.02, 0.05;
    start_xy_.a << 0.03, 0.06;

    goal_xy_.p <<  0.10, 0.11;
    goal_xy_.v <<  0.12, 0.13;
    goal_xy_.a <<  0.14, 0.15;

    start_stance_.push_back(Foothold(-0.31,  0.37, 0.0, LH));
    start_stance_.push_back(Foothold( 0.33,  0.35, 0.0, LF));
    start_stance_.push_back(Foothold(-0.35, -0.33, 0.0, RH));
    start_stance_.push_back(Foothold( 0.37, -0.31, 0.0, RF));

    times_.t_swing_ = 0.7;
    times_.t_stance_initial_ = 0.5; // this will create two initial splines
    times_.t_stance_final_ = 0.2;

    std::vector<LegID> leg_ids = {LH, LF};

    bool start_with_com_shift = true;
    xpp::zmp::ContinuousSplineContainer spline_structure;
    spline_structure.Init(start_xy_.p, start_xy_.v, leg_ids.size(),
                          times_, start_with_com_shift, true);

    SupportPolygonContainer support_polygon_container_;
    support_polygon_container_.Init(start_stance_, leg_ids, SupportPolygon::GetDefaultMargins());

    auto interpreter_ptr = std::make_shared<xpp::zmp::OptimizationVariablesInterpreter>();
    interpreter_ptr->Init(spline_structure, support_polygon_container_, robot_height_);

    nlp_facade.InitializeVariables(spline_structure.GetTotalFreeCoeff(), leg_ids.size());
    nlp_facade.SolveNlp(start_xy_.a, goal_xy_, interpreter_ptr);

    opt_xy_splines_ = nlp_facade.GetSplines();
    opt_footholds_  = nlp_facade.GetFootholds();
  }

  virtual void SetUp()
  {
    // specific setup run before each test
  }

  // these are shared over all classes (=tests) of the test-case NlpOptimizerTest
  // and derived classes
  static constexpr double robot_height_ = 0.58;
  static SplineTimes times_;
  static Point2d start_xy_, goal_xy_;
  static VecFoothold start_stance_;
  static VecSpline opt_xy_splines_;
  static VecFoothold opt_footholds_;
};




} // namespace zmp
} // namespace xpp
