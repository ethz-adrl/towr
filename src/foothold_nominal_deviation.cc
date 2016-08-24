/**
 @file    foothold_nominal_deviation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/foothold_nominal_deviation.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;
typedef Eigen::Vector2d Vector2d;


FootholdNominalDeviation::FootholdNominalDeviation ()
{
  // TODO Auto-generated constructor stub
}

// smell does two things, feet position and nomial foothold position
FootholdNominalDeviation::StdVecEigen2d
FootholdNominalDeviation::GetFeetInBase (
    const ComMotionPtr& cog_spline,
    const SupportPolygonContainer& supp_polygon_container,
    StdVecEigen2d& nominal_foothold_b_) const
{
  const double x_nominal_b = 0.36; // 0.4
  const double y_nominal_b = 0.33; // 0.4

  // this nominal position should be calculated through forward kinematics with nominal joint angles
  // SMELL This does not have to be recomputed every time
  xpp::hyq::LegDataMap<Vector2d> B_r_BaseToNominal;
  B_r_BaseToNominal[hyq::LF] <<  x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RF] <<  x_nominal_b, -y_nominal_b;
  B_r_BaseToNominal[hyq::LH] << -x_nominal_b,  y_nominal_b;
  B_r_BaseToNominal[hyq::RH] << -x_nominal_b, -y_nominal_b;

  int N     = cog_spline->GetTotalNodes();
//  int approx_n_constraints = 4*N*2; // 3 or 4 legs in contact at every discrete time, 2 b/c x-y
//  std::vector<double> g_vec;
//  g_vec.reserve(approx_n_constraints);

  StdVecEigen2d g_vec;
  g_vec.reserve(N*4); // every node has maximum 4 legs in stance
  nominal_foothold_b_.reserve(g_vec.size());

  std::vector<xpp::hyq::SupportPolygon> suppport_polygons =
      supp_polygon_container.AssignSupportPolygonsToSplines(cog_spline->GetSplines());

//  std::cout << "suppport_polygons.size(): " << suppport_polygons.size();

  double T = cog_spline->GetDiscretizedGlobalTimes().back();

  for (double t : cog_spline->GetDiscretizedGlobalTimes()) {

    // get legs in contact at each step
    VecFoothold stance_legs;
    int spline_id = cog_spline->GetSplineID(t);

    // final foothold never creates active support polygon, so handle manually
    if (t == T)
      stance_legs = supp_polygon_container.GetFinalFootholds();
    else
      stance_legs = suppport_polygons.at(spline_id).footholds_;

    xpp::utils::Point2d I_cog_xy = cog_spline->GetCom(t);

    // calculate distance to base for every stance leg
    // restrict to quadrants
    for (const hyq::Foothold& I_foot : stance_legs) {

      // base to foot in inertial frame
      Vector2d I_r_baseToFeet = I_foot.p.segment<2>(0) - I_cog_xy.p;

      // fixme this only works if the base has no yaw anlge
      Vector2d B_r_baseToFeet = I_r_baseToFeet;


      // current foot to nominal
//      Eigen::Vector2d B_r_footToCenter = -r_BF + B_r_BaseToNominal[f.leg];

      // don't use std::fabs() or std::pow() since you loose information about sign
      // this information help the optimizer to find the correct gradient.
      g_vec.push_back(B_r_baseToFeet);
//      g_vec.push_back(B_r_footToCenter);

      nominal_foothold_b_.push_back(B_r_BaseToNominal[I_foot.leg]);
    }
  }

  return g_vec;
}

//FootholdNominalDeviation::StdVecEigen2d
//FootholdNominalDeviation::GetNominalInBase () const
//{
//  return nominal_foothold_b_;
//}

} /* namespace zmp */
} /* namespace xpp */
