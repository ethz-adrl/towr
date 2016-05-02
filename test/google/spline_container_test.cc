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
protected:
  virtual void SetUp()
  {


    spline_container_4steps_.Init(Eigen::Vector2d::Zero(),
                                  Eigen::Vector2d::Zero(),
                                  step_sequence_4_,
                                  t_stance,
                                  t_swing,
                                  t_stance_initial,
                                  t_stance_final);
  }

  ContinuousSplineContainer spline_container_4steps_;

  double t_stance_initial = 2.0;
  double t_stance = 0.1;
  double t_swing = 0.5;
  double t_stance_final = 1.0;

  std::vector<LegID> step_sequence_4_ = {LH, LF, RH, RF};
};


TEST_F(SplineContainerTest, GetState)
{
  // p(t) =   at^5 +   bt^4 + ct^3  + dt^2 + et + f
  // v(t) =  5at^4 +  4bt^3 + 3ct^2 + 2dt  + e
  // a(t) = 20at^3 + 12bt^2 + 6ct   + 2d

  // Create a straight spline in x direction composed of 3 splines (4ls, step 1, 4ls)
  // that has equal position and velocity at junctions
  std::vector<ZmpSpline> x_spline;
  x_spline = ContinuousSplineContainer::ConstructSplineSequence({LH},
                                                                t_stance,
                                                                t_swing,
                                                                t_stance_initial,
                                                                t_stance_final);
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
  splines_estimated_ef.Init(init_pos,
                                init_vel,
                                {LH, LF, RH},
                                t_stance,
                                t_swing,
                                t_stance_initial,
                                t_stance_final);

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
  for (int c=A; c<= D; ++c) {
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
    f = splines_ref.at(spline-1).GetState(kPos, T.at(spline-1));
    e = splines_ref.at(spline-1).GetState(kVel, T.at(spline-1));
    coeff.x[E] = e.x();
    coeff.x[F] = f.x();
    coeff.y[E] = e.y();
    coeff.y[F] = f.y();

    splines_ref.at(spline).SetSplineCoefficients(coeff);
    for (int c=A; c<= D; ++c) {
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
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][A], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][A]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][B], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][B]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][C], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][C]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][D], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][D]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][E], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][E]);
      EXPECT_FLOAT_EQ(splines_ref.at(s).spline_coeff_[dim][F], splines_estimated_ef.GetSpline(s).spline_coeff_[dim][F]);
    }
  }
}



TEST_F(SplineContainerTest, GetSplineCount)
{
  EXPECT_EQ(1+step_sequence_4_.size()+1+1, spline_container_4steps_.GetSplineCount());
  EXPECT_EQ(6, spline_container_4steps_.GetLastSpline().GetId());


  for (const ZmpSpline& s: spline_container_4steps_.GetSplines()) {
    prt(s);
  }
}


TEST_F(SplineContainerTest, GetSplineID)
{
  EXPECT_EQ(0, spline_container_4steps_.GetSplineID(0.0));
  EXPECT_EQ(1, spline_container_4steps_.GetSplineID(t_stance_initial+0.01));
  double T = spline_container_4steps_.GetTotalTime();
  EXPECT_DOUBLE_EQ(t_stance_initial + 4*t_swing + t_stance + t_stance_final, T);

  int n_splines = 1+step_sequence_4_.size()+1+1;
  EXPECT_EQ(n_splines-1, spline_container_4steps_.GetSplineID(T));
}


TEST_F(SplineContainerTest, GetTotalFreeCoeff)
{
  int n = spline_container_4steps_.GetTotalFreeCoeff();
  int n_splines = 1+step_sequence_4_.size()+1+1;
  EXPECT_EQ(n_splines*4*2, n);
}



TEST_F(SplineContainerTest, ConstructSplineSequence)
{
  EXPECT_TRUE (spline_container_4steps_.GetSpline(0).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(1).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(2).IsFourLegSupport());
  EXPECT_TRUE (spline_container_4steps_.GetSpline(3).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(4).IsFourLegSupport());
  EXPECT_FALSE(spline_container_4steps_.GetSpline(5).IsFourLegSupport());
  EXPECT_TRUE (spline_container_4steps_.GetSpline(6).IsFourLegSupport());

  EXPECT_EQ(0,spline_container_4steps_.GetSpline(0).GetNextPlannedStep());
  EXPECT_EQ(0,spline_container_4steps_.GetSpline(1).GetCurrStep());
  EXPECT_EQ(1,spline_container_4steps_.GetSpline(2).GetCurrStep());
  EXPECT_EQ(2,spline_container_4steps_.GetSpline(3).GetNextPlannedStep());
  EXPECT_EQ(2,spline_container_4steps_.GetSpline(4).GetCurrStep());
  EXPECT_EQ(3,spline_container_4steps_.GetSpline(5).GetCurrStep());
  EXPECT_EQ(Final4lsSpline,spline_container_4steps_.GetSpline(6).GetType());
}



} // namespace hyq
} // namespace xpp
