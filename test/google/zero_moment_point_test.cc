/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/zmp/zero_moment_point.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;


namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper;

class ZeroMomentPointTest : public ::testing::Test {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // for fixed size eigen members
  typedef ZeroMomentPoint::MatVec MatVec;

protected:
  virtual void SetUp()
  {
    init_pos.setZero();
    init_vel.setZero();

    init_pos << 0.8, 1.2;
    init_vel << 0.2, 1.3;

    cont_spline_container_.Init(init_pos,
                                init_vel,
                                3,
                                SplineTimes(t_swing, t_stance_initial));
  }

  ComSpline4 cont_spline_container_;
  double t_stance_initial = 1.2;
  double t_swing = 0.75; // muss .2, .4, .6, ..sein
  double walking_height = 0.58;

  Eigen::Vector2d init_pos;
  Eigen::Vector2d init_vel;
};


TEST_F(ZeroMomentPointTest, ExpressZmpThroughCoefficients)
{
  // create a random spline with pos, vel equal at spline juntions
  std::vector<ZmpSpline> splines = cont_spline_container_.GetSplines(); // get ids and durations
  Eigen::VectorXd abcd(cont_spline_container_.GetTotalFreeCoeff());
  CoeffValues coeff;

  // initialize first spline to match position and velocity
  // all other splines must match position, velocity at junctions
  // build first spline
  coeff.SetRandom();
  coeff.x[E] = init_vel.x();
  coeff.x[F] = init_pos.x();
  coeff.y[E] = init_vel.y();
  coeff.y[F] = init_pos.y();
  splines.front().SetSplineCoefficients(coeff);
  for (const SplineCoeff c : FreeSplineCoeff) {
    abcd(ComSpline4::Index(0, X, c)) = coeff.x[c];
    abcd(ComSpline4::Index(0, Y, c)) = coeff.y[c];
  }

  // shorthand for spline durations
  const int n_splines = splines.size();
  std::vector<double> T;
  for (int spline=0; spline<n_splines; spline++)
    T.push_back(splines.at(spline).GetDuration());


  Eigen::Vector2d f,e;
  // build splines ensuring equal position and velocity at juntions
  for (int spline=1; spline<n_splines; spline++) {

    CoeffValues coeff;
    coeff.SetRandom();

    // make sure splines are continuous in position and velocity
    f = splines.at(spline-1).GetState(xpp::utils::kPos, T.at(spline-1));
    e = splines.at(spline-1).GetState(xpp::utils::kVel, T.at(spline-1));
    coeff.x[E] = e.x();
    coeff.x[F] = f.x();
    coeff.y[E] = e.y();
    coeff.y[F] = f.y();

    splines.at(spline).SetSplineCoefficients(coeff);
    for (const SplineCoeff c : FreeSplineCoeff) {
      abcd(ComSpline4::Index(spline, X, c)) = coeff.x[c];
      abcd(ComSpline4::Index(spline, Y, c)) = coeff.y[c];
    }
  }


  // expresses zero moment point only depending on initial pos and velocity
  // and a,b,c,d coefficients of spline
  auto ZmpMap = &ZeroMomentPoint::ExpressZmpThroughCoefficients;
  MatVec x_zmp_map = ZmpMap(cont_spline_container_,walking_height,X);
  MatVec y_zmp_map = ZmpMap(cont_spline_container_,walking_height,Y);
  // with the initial conditions and the abcd coefficients, calculate the zmp
  // for every discrete time step
  Eigen::VectorXd x_zmp = x_zmp_map.M*abcd + x_zmp_map.v;
  Eigen::VectorXd y_zmp = y_zmp_map.M*abcd + y_zmp_map.v;
  prt(x_zmp.rows());


  // compare
  double t = 0.0;
  int n = 0;
  for (double t : cont_spline_container_.GetDiscretizedGlobalTimes())
  {
    xpp::utils::Point2d cog_xy = ComSpline4::GetCOGxy(t, splines);
    Eigen::Vector2d zmp_true = ZeroMomentPoint::CalcZmp(cog_xy.Make3D(), walking_height);

    SCOPED_TRACE("n = " + std::to_string(n));
    EXPECT_FLOAT_EQ(zmp_true.x(), x_zmp[n]);
    EXPECT_FLOAT_EQ(zmp_true.y(), y_zmp[n]);

    n++;
  }





//  EXPECT_EQ(f_top_left.size(), poly.footholds_conv_.size());
}







} // namespace hyq
} // namespace xpp
