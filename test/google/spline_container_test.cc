/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/spline_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::hyq;
using namespace xpp::utils::coords_wrapper;

class SplineContainerTest : public ::testing::Test {

public:
  typedef SplineContainer::VecSpline VecSpline;

protected:
  virtual void SetUp()
  {

    // create a spline container taking four steps
    times_.t_stance_ = 0.1;
    times_.t_stance_final_ = 0.6;
    times_.t_swing_ = 0.6;
    times_.t_stance_initial_ = 2.0;

    spline_container_4steps_.Init(Eigen::Vector2d::Zero(),
                                  Eigen::Vector2d::Zero(),
                                  {LH, LF, RH, RF},
                                  times_);

    n_steps = 4;
    n_four_leg_support = 1; // LF->RH
    n_initial_splines  = std::ceil(times_.t_stance_initial_/0.4);
    n_final_splines    = std::ceil(times_.t_stance_final_/0.4);
  }

  ContinuousSplineContainer spline_container_4steps_;
  SplineTimes times_;

  int n_initial_splines;
  int n_steps;
  int n_four_leg_support;
  int n_final_splines;
};


TEST_F(SplineContainerTest, ConstructSplineSequenceInitFinalCount)
{
  VecSpline spline = ContinuousSplineContainer::ConstructSplineSequence({LH, LF}, times_);

  // count initial and final polynoms that correspond to a four leg support phase
  int init_polys  = 0;
  int final_polys = 0;
  for (const ZmpSpline& s : spline) {
    if (s.GetType() == Initial4lsSpline)
      init_polys++;
    if (s.GetType() == Final4lsSpline)
      final_polys++;
  }

  EXPECT_EQ(n_initial_splines, init_polys);
  EXPECT_EQ(n_final_splines, final_polys);
  EXPECT_EQ(init_polys + 2/*steps*/ + 0/*4LS*/ + final_polys, spline.size());
}


TEST_F(SplineContainerTest, GetSplineCount)
{
  int n_total = n_initial_splines + n_steps + n_four_leg_support + n_final_splines;
  EXPECT_EQ(n_total, spline_container_4steps_.GetSplineCount());
}


TEST_F(SplineContainerTest, GetTotalFreeCoeff)
{
  int n_splines = spline_container_4steps_.GetSplineCount();
  int n = n_splines * kFreeCoeffPerSpline * utils::kDim2d;
  EXPECT_EQ(n, spline_container_4steps_.GetTotalFreeCoeff());
}


TEST_F(SplineContainerTest, GetTotalTime)
{
  double T =  1                 *times_.t_stance_initial_
            + n_steps           *times_.t_swing_
            + n_four_leg_support*times_.t_stance_
            + 1                 *times_.t_stance_final_;

  EXPECT_DOUBLE_EQ(T, spline_container_4steps_.GetTotalTime());
}


TEST_F(SplineContainerTest, GetSplineID)
{
  EXPECT_EQ(0, spline_container_4steps_.GetSplineID(0.0));
  EXPECT_EQ(n_initial_splines, spline_container_4steps_.GetSplineID(times_.t_stance_initial_+0.01));

  double T = spline_container_4steps_.GetTotalTime();
  int last_id = spline_container_4steps_.GetLastSpline().GetId();

  EXPECT_EQ(last_id  , spline_container_4steps_.GetSplineID(T));
  EXPECT_EQ(last_id  , spline_container_4steps_.GetSplineID(T-0.05));
  double t_just_at_end_of_swing = T-times_.t_stance_final_;
  EXPECT_EQ(last_id-n_final_splines, spline_container_4steps_.GetSplineID(t_just_at_end_of_swing));
  EXPECT_EQ(last_id-n_final_splines-1, spline_container_4steps_.GetSplineID(t_just_at_end_of_swing - times_.t_swing_ - 0.01));
}


TEST_F(SplineContainerTest, GetLocalTime)
{
  double t_global;
  t_global = 0.1; // right at start, so first spline
  EXPECT_FLOAT_EQ(0.1, spline_container_4steps_.GetLocalTime(t_global));

  t_global = times_.t_stance_initial_+0.1; // right after starting to swing first leg
  EXPECT_FLOAT_EQ(0.1, spline_container_4steps_.GetLocalTime(t_global));
  EXPECT_FLOAT_EQ(0.2, spline_container_4steps_.GetLocalTime(t_global+0.1));

  t_global = spline_container_4steps_.GetTotalTime();
  EXPECT_FLOAT_EQ(times_.t_stance_final_/n_final_splines, spline_container_4steps_.GetLocalTime(t_global));
}


TEST_F(SplineContainerTest, IsFourLegSupport)
{
  int id = 0;
  SCOPED_TRACE("id" + id);

  for (int j=0; j<n_initial_splines; ++j)
    EXPECT_TRUE (spline_container_4steps_.GetSpline(id++).IsFourLegSupport());

  EXPECT_FALSE(spline_container_4steps_.GetSpline(id++).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(id++).IsFourLegSupport());
  EXPECT_TRUE (spline_container_4steps_.GetSpline(id++).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(id++).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(id++).IsFourLegSupport());
  EXPECT_TRUE (spline_container_4steps_.GetSpline(id++).IsFourLegSupport());

}


TEST_F(SplineContainerTest, GetNextPlannedStep)
{
  int id = 0;
  SCOPED_TRACE("id" + id);

  for (int j=0; j<n_initial_splines; ++j)
    EXPECT_EQ(0, spline_container_4steps_.GetSpline(id++).GetNextPlannedStep());

  EXPECT_EQ(0,spline_container_4steps_.GetSpline(id++).GetCurrStep());
  EXPECT_EQ(1,spline_container_4steps_.GetSpline(id++).GetCurrStep());
  EXPECT_EQ(2,spline_container_4steps_.GetSpline(id++).GetNextPlannedStep());
  EXPECT_EQ(2,spline_container_4steps_.GetSpline(id++).GetCurrStep());
  EXPECT_EQ(3,spline_container_4steps_.GetSpline(id++).GetCurrStep());
  EXPECT_EQ(Final4lsSpline,spline_container_4steps_.GetSpline(id++).GetType());

}


TEST_F(SplineContainerTest, GetState)
{
  // p(t) =   at^5 +   bt^4 + ct^3  + dt^2 + et + f
  // v(t) =  5at^4 +  4bt^3 + 3ct^2 + 2dt  + e
  // a(t) = 20at^3 + 12bt^2 + 6ct   + 2d

  // Create a straight spline in x direction composed of 3 splines (4ls, step 1, 4ls)
  // that has equal position and velocity at junctions
  std::vector<ZmpSpline> x_spline = ContinuousSplineContainer::ConstructSplineSequence({LH}, times_);

  double pos0 = 0.4; // initial position (f0)
  double vel0 = 1.1; // initial velocity (e0)
  double acc0 = 3.1; // initial acceleration (2*d0)

  double f0 = pos0;
  double e0 = vel0;
  double d0 = acc0/2.;
  CoeffValues coeff;

  int i=0;
  // first spline
  coeff = CoeffValues(0.0, 0.0, 0.0, d0, e0, f0,        /* x: a -> f */
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0);    // y: a -> f
  x_spline.at(i++).SetSplineCoefficients(coeff);
  double T0 = x_spline.at(0).GetDuration();

  // second spline
  double f1 = f0 + e0*T0 +   d0*std::pow(T0,2);
  double e1 =      e0    + 2*d0*T0;
  double d1 = 0.0; // arbitrary
  coeff = CoeffValues(0.0, 0.0, 0.0, d1, e1, f1,       /* x: a -> f */
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   // y: a -> f
  x_spline.at(i++).SetSplineCoefficients(coeff);
  double T1 = x_spline.at(1).GetDuration();

  // last spline
  double f2 = f1 + e1*T1 +   d1*std::pow(T1,2);
  double e2 =      e1    + 2*d1*T1;
  double d2 = 0.0; // arbitrary
  coeff = CoeffValues(0.0, 0.0, 0.0, d2, e2, f2,     /* x: a -> f */
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // y: a -> f
  x_spline.at(i++).SetSplineCoefficients(coeff);


  // initial position and velocity
  using namespace xpp::utils;
  EXPECT_DOUBLE_EQ(pos0, x_spline.at(0).GetState(kPos, 0.0).x());
  EXPECT_DOUBLE_EQ(vel0, x_spline.at(0).GetState(kVel, 0.0).x());

  // spline junction 0->1 pos/vel
  EXPECT_DOUBLE_EQ(x_spline.at(1).GetState(kPos, 0.0).x(),
                   x_spline.at(0).GetState(kPos, T0).x());
  EXPECT_DOUBLE_EQ(x_spline.at(1).GetState(kVel, 0.0).x(),
                   x_spline.at(0).GetState(kVel, T0).x());

  // spline junction 1->2 pos/vel
  EXPECT_DOUBLE_EQ(x_spline.at(2).GetState(kPos, 0.0).x(),
                   x_spline.at(1).GetState(kPos, T1).x());
  EXPECT_DOUBLE_EQ(x_spline.at(2).GetState(kVel, 0.0).x(),
                   x_spline.at(1).GetState(kVel, T1).x());


}



TEST_F(SplineContainerTest, EandFCoefficientTest)
{
  // p(t) =   at^5 +   bt^4 + ct^3  + dt^2 + et + f
  // v(t) =  5at^4 +  4bt^3 + 3ct^2 + 2dt  + e
  // a(t) = 20at^3 + 12bt^2 + 6ct   + 2d

  Eigen::Vector2d init_pos, init_vel, init_acc;
  init_pos.setZero(); init_vel.setZero();
  init_pos << 0.4, 3.4;
  init_vel << 1.1, -1.9;

  ContinuousSplineContainer splines_estimated_ef;
  splines_estimated_ef.Init(init_pos, init_vel, {LH, LF, RH}, times_);

  // Create a straight spline in x direction composed of 3 splines (4ls, step 1, 4ls)
  // that has equal position and velocity at junctions
  std::vector<ZmpSpline> splines_ref = splines_estimated_ef.GetSplines();
  Eigen::VectorXd abcd_coeff(splines_estimated_ef.GetTotalFreeCoeff());
  abcd_coeff.setZero();

  Eigen::Vector2d f,e,d;


  int spline=0;
  f = init_pos;
  e = init_vel;
  init_acc << 3.1, -0.5;
  d = init_acc/2.0;


  // build first spline
  CoeffValues coeff;
  coeff.SetRandom();

  coeff.x[D] = d.x();
  coeff.x[E] = e.x();
  coeff.x[F] = f.x();

  coeff.y[D] = d.y();
  coeff.y[E] = e.y();
  coeff.y[F] = f.y();
  splines_ref.at(spline).SetSplineCoefficients(coeff);
  for (const SplineCoeff c : FreeSplineCoeff) {
    abcd_coeff(ContinuousSplineContainer::Index(spline, X, c)) = coeff.x[c];
    abcd_coeff(ContinuousSplineContainer::Index(spline, Y, c)) = coeff.y[c];
  }


  // shorthand for spline durations
  const int n_splines = splines_estimated_ef.GetSplineCount();
  std::vector<double> T;
  for (int spline=0; spline<n_splines; spline++)
    T.push_back(splines_ref.at(spline).GetDuration());


  // build splines ensuring equal position and velocity at juntions
  for (int spline=1; spline<n_splines; spline++) {

    CoeffValues coeff;
    coeff.SetRandom();

    // make sure splines are continuous in position and velocity
    f = splines_ref.at(spline-1).GetState(xpp::utils::kPos, T.at(spline-1));
    e = splines_ref.at(spline-1).GetState(xpp::utils::kVel, T.at(spline-1));
    coeff.x[E] = e.x();
    coeff.x[F] = f.x();
    coeff.y[E] = e.y();
    coeff.y[F] = f.y();

    splines_ref.at(spline).SetSplineCoefficients(coeff);
    for (const SplineCoeff c : FreeSplineCoeff) {
      abcd_coeff(ContinuousSplineContainer::Index(spline, X, c)) = coeff.x[c];
      abcd_coeff(ContinuousSplineContainer::Index(spline, Y, c)) = coeff.y[c];
    }
  }


  // Describe the same spline with only abcd coefficients and the constraint that
  // the junctions should have equal position and velocity
  splines_estimated_ef.AddOptimizedCoefficients(abcd_coeff);

  for (int dim=X; dim<=Y; ++dim) {
    EXPECT_FLOAT_EQ(init_pos[dim], splines_estimated_ef.GetSpline(0).spline_coeff_[dim][F]);
    EXPECT_FLOAT_EQ(init_vel[dim], splines_estimated_ef.GetSpline(0).spline_coeff_[dim][E]);
    EXPECT_FLOAT_EQ(init_acc[dim], 2* splines_estimated_ef.GetSpline(0).spline_coeff_[dim][D]);
  }


  // the set coefficients and the one estimated through above constraint must be equal
  for (int s=1; s<n_splines; s++) {
    for (int dim=X; dim<=Y; ++dim) {
      SCOPED_TRACE("s = " + std::to_string(s));
      SCOPED_TRACE("dim = " + std::to_string(dim));
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][A], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][A]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][B], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][B]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][C], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][C]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][D], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][D]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][E], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][E]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][F], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][F]);
    }
  }
}



} // namespace hyq
} // namespace xpp
