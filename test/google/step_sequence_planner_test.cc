/**
 @file    step_sequence_planner_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Brief description
 */

#include <xpp/hyq/step_sequence_planner.h>
#include <gtest/gtest.h>

namespace xpp {
namespace hyq {

class StepSequencePlannerTest : public ::testing::Test {
public:
  typedef StepSequencePlanner::State State;
  typedef StepSequencePlanner::LegIDVec LegIDVec;
  typedef StepSequencePlanner::StartStance VecFoothold;

protected:
  virtual void SetUp() {
    State curr;
    State goal;
    goal.p.x() = 0.25;

    VecFoothold curr_stance;
    curr_stance.push_back(Foothold( 0.35,  0.3, 0.0, LF));
    curr_stance.push_back(Foothold( 0.35, -0.3, 0.0, RF));
    curr_stance.push_back(Foothold(-0.35,  0.3, 0.0, LH));
    curr_stance.push_back(Foothold(-0.35, -0.3, 0.0, RH));

    double robot_height = 0.58;

    planner_.Init(curr, goal, curr_stance, robot_height);
  }

  StepSequencePlanner planner_;
};

TEST_F(StepSequencePlannerTest, DetermineStepSequence)
{
  int curr_swingleg = LF;
  LegIDVec step_sequence = planner_.DetermineStepSequence(curr_swingleg);

  EXPECT_EQ(4, step_sequence.size());
  EXPECT_EQ(RH, step_sequence[0]);
  EXPECT_EQ(RF, step_sequence[1]);
  EXPECT_EQ(LH, step_sequence[2]);
  EXPECT_EQ(LF, step_sequence[3]);
}

TEST_F(StepSequencePlannerTest, StartWithStancePhase)
{
  bool start_with_com_shift = planner_.StartWithStancePhase({LF, RF});

  EXPECT_TRUE(start_with_com_shift);
}

} /* namespace hyq */
} /* namespace xpp */
