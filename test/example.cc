/*!
 * \file   example.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 4, 2014
 * \brief  An example implementation of how to generate a trajectory.
 */

#include <xpp/zmp/zmp_optimizer.h>

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/basicconfigurator.h>

#include <Eigen/Dense>
#include <iostream> //std::cout, std::fixed
#include <iomanip>  //std::setprecision


int main() 
{
  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;

  log4cxx::PropertyConfigurator::configure("../test/log4cxx.properties");
  log4cxx::LoggerPtr main_logger = log4cxx::Logger::getLogger("main");

  int splines_per_step = 2;
  int splines_per_4ls = 1;

  double penalty_movement_x = 1.0;
  double penalty_movement_y = 1.5;
  ZmpOptimizer::WeightsXYArray weight = {{penalty_movement_x, penalty_movement_y}};

  MarginValues margins;
  margins[FRONT] = 0.1;
  margins[HIND]  = 0.1;
  margins[SIDE]  = 0.1;
  margins[DIAG]  = 0.05; // controls sidesway motion

  double discretization_time = 0.1; 
  double swing_time = 0.6;         
  double stance_time = 0.2;         

  // set up the general attributes of the optimizer
  ZmpOptimizer opt(discretization_time,
                   splines_per_step, splines_per_4ls,
                   swing_time, stance_time);

  // start position (x,y,z) of robot
  Eigen::Vector2d cog_start_p(0.0, 0.0);
  Eigen::Vector2d cog_start_v(0.0, 0.0);
  LegDataMap<Foothold> start_stance;
  start_stance[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  start_stance[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  start_stance[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  start_stance[RH] = Foothold(-0.35, -0.3, 0.0, RH);
  double t_stance_initial = 1.0; //s

  // steps in global reference frame
  std::vector<Foothold> steps;
  steps.push_back(Foothold(-0.25,  0.3, 0.0, LH));
  steps.push_back(Foothold( 0.50,  0.3, 0.0, LF));
  steps.push_back(Foothold(-0.12, -0.3, 0.0, RH));
  steps.push_back(Foothold( 0.62, -0.3, 0.0, RF));
  steps.push_back(Foothold( 0.01,  0.3, 0.0, LH));
  steps.push_back(Foothold( 0.75,  0.3, 0.0, LF));
  steps.push_back(Foothold( 0.14, -0.3, 0.0, RH));
  steps.push_back(Foothold( 0.88, -0.3, 0.0, RF));

  double robot_height = 0.58;

  std::vector<ZmpSpline> spline_coefficients;
  ////////////////// QP optimization using eigen_quadprog /////////////////////
  opt.OptimizeSplineCoeff(cog_start_p, cog_start_v, start_stance, t_stance_initial,
                          steps, weight, margins, robot_height,
                          spline_coefficients);
  /////////////////////////////////////////////////////////////////////////////


  SplineContainer zmp_splines;
  zmp_splines.AddSplines(spline_coefficients);
  LOG4CXX_INFO(main_logger, "\nZMP-optimized CoG Trajectory:\n"
               << "position(p), velocity(v), acclerations(a) [x,y]");

  for (double t(0.0); t < swing_time*steps.size(); t+= 0.2)
  {
    Point2d cog_state;
    zmp_splines.GetCOGxy(t, cog_state);
    LOG4CXX_INFO(main_logger, "t = " << t << "s:\t"
                             << std::setprecision(2) << std::fixed
                             << cog_state );
  }
}


