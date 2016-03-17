/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#include <xpp/zmp/zmp_optimizer.h>
#include <xpp/zmp/eigen_quadprog-inl.h>
#include <xpp/hyq/supp_triangle.h>
#include <xpp/utils/logger_helpers-inl.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <ctime>      // std::clock_t
#include <cmath>      // std::numeric_limits
#include <algorithm>  // std::count

namespace xpp {
namespace zmp {

using hyq::LF; using hyq::RF; using hyq::LH; using hyq::RH;
using utils::X; using utils::Y; using utils::Z;

log4cxx::LoggerPtr  ZmpOptimizer::log_ = log4cxx::Logger::getLogger("xpp.zmp.zmpoptimizer");
log4cxx::LoggerPtr  ZmpOptimizer::log_matlab_ = log4cxx::Logger::getLogger("matlab");

ZmpOptimizer::ZmpOptimizer()
{
  LOG4CXX_WARN(log_, "default params are set!");
}


ZmpOptimizer::~ZmpOptimizer() {}


void ZmpOptimizer::SetupQpMatrices(
    const Position &start_cog_p,
    const Velocity &start_cog_v,
    const hyq::LegDataMap<Foothold>& start_stance,
    const Footholds& steps,
    const WeightsXYArray& weight, hyq::MarginValues margins, double height_robot)
{
  if (zmp_splines_.empty()) {
    throw std::runtime_error("zmp.optimizer.cc: spline_info vector empty. First call ConstructSplineSequence()");
  }

  cf_ = CreateMinAccCostFunction(weight);

  hyq::LegDataMap<Foothold> final_stance;
  footholds_ = steps;
  SuppTriangles tr = SuppTriangle::FromFootholds(start_stance, steps, margins, final_stance);



  // calculate average(x,y) of last stance to move robot in the end
  Position end_cog = Position::Zero();
  for (LegID leg : hyq::LegIDArray) end_cog += final_stance[leg].p.segment<kDim2d>(X);
  end_cog /= 4; // number of legs

  eq_ = CreateEqualityContraints(start_cog_p, start_cog_v, end_cog);

  double dt = 0.1;
  lines_for_constraint_ = LineForConstraint(tr, dt);
  ineq_ = CreateInequalityContraints(start_cog_p, start_cog_v, lines_for_constraint_, height_robot, dt);
//  AddLineDependencies(ineq_, start_cog_p, start_cog_v, tr);
}


Eigen::VectorXd ZmpOptimizer::SolveQp() {

  Eigen::VectorXd opt_spline_coeff_xy(zmp_splines_.size() * kOptCoeff * kDim2d);

  clock_t start = std::clock();

  double cost = Eigen::solve_quadprog(cf_.M, cf_.v, eq_.M, eq_.v, ineq_.M, ineq_.v,
                                      opt_spline_coeff_xy);
  clock_t end = std::clock();

  LOG4CXX_INFO(log_, "Time QP solver:\t\t" << static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0 << "\tms");
  LOG4CXX_INFO(log_, "Cost:\t\t" << cost);
  LOG4CXX_INFO(log_matlab_, opt_spline_coeff_xy.transpose());


  if (cost == std::numeric_limits<double>::infinity() || cost < 0.002)
    throw std::length_error("Eigen::quadprog did not find a solution");

  LOG4CXX_TRACE(log_, "x = " << opt_spline_coeff_xy.transpose()); //ax1, bx1, cx1, dx1, ex1, fx1 -- ay1, by1, cy1, dy1, ey1, fy1 -- ax2, bx2, cx2, dx2, ex2, fx2 -- ay2, by2, cy2, dy2, ey2, fy2 ...
  return opt_spline_coeff_xy;
}


Eigen::VectorXd ZmpOptimizer::SolveIpopt(Eigen::VectorXd& final_footholds,
                                         const Eigen::VectorXd& opt_coefficients_eig)
{



  Ipopt::IpoptApplication app;
  Ipopt::ApplicationReturnStatus status = app.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }


  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> nlp_ipopt_zmp = new Ipopt::NlpIpoptZmp();
  nlp_ipopt_zmp->SetupNlp(cf_,eq_,
                          ineq_ipopt_, ineq_ipopt_vx_, ineq_ipopt_vy_, lines_for_constraint_,
                          *this, opt_coefficients_eig);


  // FIXME make sure the zmp_optimizer member variables is already properly filled!!!
  status = app.OptimizeTNLP(nlp_ipopt_zmp);
  if (status == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;

  }

  final_footholds = nlp_ipopt_zmp->x_final_footholds_;
  return nlp_ipopt_zmp->x_final_spline_coeff_;
}


// Creates a sequence of Splines without the optimized coefficients
const ZmpOptimizer::Splines
ZmpOptimizer::ConstructSplineSequence(const std::vector<LegID>& step_sequence,
                                      double t_stance,
                                      double t_swing,
                                      double t_stance_initial,
                                      double t_stance_final)
{
  zmp_splines_.clear();
  leg_ids_ = step_sequence;
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  const int kSplinesPer4ls = 1;
  const int kSplinesPerStep = 1;

  for (size_t i = 0; i < step_sequence.size(); ++i)
  {
    // 1. insert 4ls-phase when switching between disjoint support triangles
    // Attention: these 4ls-phases much coincide with the ones in the zmp optimizer
    if (i==0) {
      zmp_splines_.push_back(ZmpSpline(id++, t_stance_initial, true, step));
    } else {
      LegID swing_leg = step_sequence[i];
      LegID swing_leg_prev = step_sequence[i-1];
      if (SuppTriangle::Insert4LSPhase(swing_leg_prev, swing_leg))
        for (int s = 0; s < kSplinesPer4ls; s++)
          zmp_splines_.push_back(ZmpSpline(id++, t_stance/kSplinesPer4ls, true, step));
    }


    // insert swing phase splines
    for (int s = 0; s < kSplinesPerStep; s++)
      zmp_splines_.push_back(ZmpSpline(id++, t_swing/kSplinesPerStep, false, step));



    // always have last 4ls spline for robot to move into center of feet
    if (i==step_sequence.size()-1)
      zmp_splines_.push_back(ZmpSpline(id++, t_stance_final, true, step));

    step++;
  }


//  ::xpp::utils::logger_helpers::print_spline_info(spline_infos_, log_);
//  LOG4CXX_INFO(log_matlab_, step);
//  LOG4CXX_INFO(log_matlab_, (t_swing/kSplinesPerStep) << " " << t_stance);
  return zmp_splines_;
}


xpp::zmp::MatVec
ZmpOptimizer::CreateMinAccCostFunction(const WeightsXYArray& weight) const
{
  std::clock_t start = std::clock();

  // total number of coefficients to be optimized
  int n_coeff = zmp_splines_.size() * kOptCoeff * kDim2d;
  MatVec cf(n_coeff, n_coeff);

  for (const ZmpSpline& s : zmp_splines_) {
    std::array<double,10> t_span = cache_exponents<10>(s.duration_);

    for (int dim = X; dim <= Y; dim++) {
      const int a = var_index(s.id_, dim, A);
      const int b = var_index(s.id_, dim, B);
      const int c = var_index(s.id_, dim, C);
      const int d = var_index(s.id_, dim, D);

      // for explanation of values see M.Kalakrishnan et al., page 248
      // "Learning, Planning and Control for Quadruped Robots over challenging
      // Terrain", IJRR, 2010
      cf.M(a, a) = 400.0 / 7.0      * t_span[7] * weight[dim];
      cf.M(a, b) = 40.0             * t_span[6] * weight[dim];
      cf.M(a, c) = 120.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(a, d) = 10.0             * t_span[4] * weight[dim];
      cf.M(b, b) = 144.0 / 5.0      * t_span[5] * weight[dim];
      cf.M(b, c) = 18.0             * t_span[4] * weight[dim];
      cf.M(b, d) = 8.0              * t_span[3] * weight[dim];
      cf.M(c, c) = 12.0             * t_span[3] * weight[dim];
      cf.M(c, d) = 6.0              * t_span[2] * weight[dim];
      cf.M(d, d) = 4.0              * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < cf.M.cols(); ++c)
        for (int r = c + 1; r < cf.M.rows(); ++r)
          cf.M(r, c) = cf.M(c, r);
    }
  }

  LOG4CXX_INFO(log_, "Calc. time cost function:\t\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << cf.M << "\nVector:\n" << cf.v.transpose());
  return cf;
}

xpp::zmp::MatVec
ZmpOptimizer::CreateEqualityContraints(const Position &start_cog_p,
                                       const Velocity &start_cog_v,
                                       const Position &end_cog) const
{
  std::clock_t start = std::clock();

  int coeff = zmp_splines_.size() * kOptCoeff * kDim2d; // total number of all spline coefficients
  int constraints = 0;
  constraints += kDim2d*2;                         // init {x,y} * {acc, jerk} pos, vel implied
  constraints += kDim2d*3;                         // end  {x,y} * {pos, vel, acc}
  constraints += (zmp_splines_.size()-1) * kDim2d * 2;  // junctions {acc,jerk} since pos, vel  implied
  MatVec ec(coeff, constraints);

  const Eigen::Vector2d kAccStart = Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kJerkStart= Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kVelEnd   = Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d kAccEnd   = Eigen::Vector2d(0.0, 0.0);


  int i = 0; // counter of equality constraints
  for (int dim = X; dim <= Y; ++dim)
  {
    // acceleration set to zero
    int d = var_index(0, dim, D);
    ec.M(d, i) = 2.0;
    ec.v(i++) = -kAccStart(dim);
    // jerk set to zero
    int c = var_index(0, dim, C);
    ec.M(c, i) = 6.0;
    ec.v(i++) = -kJerkStart(dim);


    // 2. Final conditions
    int K = zmp_splines_.back().id_; // id of last spline
    int last_spline = var_index(K, dim, A);
    std::array<double,6> t_duration = cache_exponents<6>(zmp_splines_.back().duration_);

    // calculate e and f coefficients from previous values
    Eigen::VectorXd Ek(coeff); Ek.setZero();
    Eigen::VectorXd Fk(coeff); Fk.setZero();
    double non_dependent_e, non_dependent_f;
    DescribeEByPrev(K, dim, start_cog_v(dim), Ek, non_dependent_e);
    DescribeFByPrev(K, dim, start_cog_v(dim), start_cog_p(dim), Fk, non_dependent_f);

    // position
    ec.M(last_spline + A, i) = t_duration[5];
    ec.M(last_spline + B, i) = t_duration[4];
    ec.M(last_spline + C, i) = t_duration[3];
    ec.M(last_spline + D, i) = t_duration[2];
    ec.M.col(i) += Ek*t_duration[1];
    ec.M.col(i) += Fk;

    ec.v(i)     += non_dependent_e*t_duration[1] + non_dependent_f;
    ec.v(i++)   += -end_cog(dim);

    // velocities
    ec.M(last_spline + A, i) = 5 * t_duration[4];
    ec.M(last_spline + B, i) = 4 * t_duration[3];
    ec.M(last_spline + C, i) = 3 * t_duration[2];
    ec.M(last_spline + D, i) = 2 * t_duration[1];
    ec.M.col(i) += Ek;

    ec.v(i)     += non_dependent_e;
    ec.v(i++)   += -kVelEnd(dim);

    // accelerations
    ec.M(last_spline + A, i) = 20 * t_duration[3];
    ec.M(last_spline + B, i) = 12 * t_duration[2];
    ec.M(last_spline + C, i) = 6  * t_duration[1];
    ec.M(last_spline + D, i) = 2;

    ec.v(i++) = -kAccEnd(dim);
  }

  // 3. Equal conditions at spline junctions
  for (Splines::size_type s = 0; s < zmp_splines_.size() - 1; ++s)
  {
    std::array<double,6> t_duration = cache_exponents<6>(zmp_splines_.at(s).duration_);
    for (int dim = X; dim <= Y; dim++) {

      int curr_spline = var_index(s, dim, A);
      int next_spline = var_index(s + 1, dim, A);

      // acceleration
      ec.M(curr_spline + A, i) = 20 * t_duration[3];
      ec.M(curr_spline + B, i) = 12 * t_duration[2];
      ec.M(curr_spline + C, i) = 6  * t_duration[1];
      ec.M(curr_spline + D, i) = 2;
      ec.M(next_spline + D, i) = -2.0;
      ec.v(i++) = 0.0;

      // jerk (derivative of acceleration)
      ec.M(curr_spline + A, i) = 60 * t_duration[2];
      ec.M(curr_spline + B, i) = 24 * t_duration[1];
      ec.M(curr_spline + C, i) = 6;
      ec.M(next_spline + C, i) = -6.0;
      ec.v(i++) = 0.0;
    }
  }

  LOG4CXX_INFO(log_, "Calc. time equality constraints:\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
  LOG4CXX_DEBUG(log_, "Dim: " << ec.M.rows() << " x " << ec.M.cols());
  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << ec.M << "\nVector:\n" << ec.v.transpose());
  return ec;
}



xpp::zmp::MatVec
ZmpOptimizer::CreateInequalityContraints(const Position& start_cog_p,
                                         const Velocity& start_cog_v,
                                         const std::vector<SuppTriangle::TrLine> &line_for_constraint,
                                         double h,
                                         double dt)
{
  std::clock_t start = std::clock();

  int coeff = zmp_splines_.size() * kOptCoeff * kDim2d;

  MatVec ineq(coeff, line_for_constraint.size());
  ineq_ipopt_ = ineq.M;
  ineq_ipopt_vx_ = ineq.v;
  ineq_ipopt_vy_ = ineq.v;

  const double g = 9.81; // gravity acceleration
  int c = 0; // inequality constraint counter

  for (const ZmpSpline& s : zmp_splines_) {
    LOG4CXX_TRACE(log_, "Calc inequality constaints of spline " << s.id_ << " of " << zmp_splines_.size() << ", duration=" << std::setprecision(3) << s.duration_ << ", step=" << s.step_);

    // no constraints in 4ls phase
    if (s.four_leg_supp_) continue;
    // TODO: insert support polygon constraints here instead of just allowing
    // the ZMP to be anywhere.

    // calculate e and f coefficients from previous values
    const int k = s.id_;
    Eigen::VectorXd Ekx(coeff); Ekx.setZero();
    Eigen::VectorXd Fkx(coeff); Fkx.setZero();
    Eigen::VectorXd Eky(coeff); Eky.setZero();
    Eigen::VectorXd Fky(coeff); Fky.setZero();
    double non_dependent_ex, non_dependent_fx, non_dependent_ey, non_dependent_fy;
    DescribeEByPrev(k, X, start_cog_v(X), Ekx, non_dependent_ex);
    DescribeFByPrev(k, X, start_cog_v(X), start_cog_p(X), Fkx, non_dependent_fx);
    DescribeEByPrev(k, Y, start_cog_v(Y), Eky, non_dependent_ey);
    DescribeFByPrev(k, Y, start_cog_v(Y), start_cog_p(Y), Fky, non_dependent_fy);

    for (double i=0; i < std::floor(s.duration_/dt); ++i) {

      double time = i*dt;

      std::array<double,6> t = cache_exponents<6>(time);

      // one constraint per line of support triangle
      for (int j=0; j<3; ++j) {
        // the zero moment point must always lay on one side of triangle side:
        // p*x_zmp + q*y_zmp + r > stability_margin
        // with: x_zmp = x_pos - height/(g+z_acc) * x_acc
        //       x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
        //       x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
        SuppTriangle::TrLine l = line_for_constraint.at(c);
        double z_acc = 0.0; // TODO: calculate z_acc based on foothold height


        for (int dim=X; dim<=Y; dim++) {

          double lc = (dim==X) ? l.coeff.p : l.coeff.q;
          Eigen::VectorXd Ek = (dim==X) ? Ekx : Eky;
          Eigen::VectorXd Fk = (dim==X) ? Fkx : Fky;


          ineq_ipopt_(var_index(k,dim,A), c) = /* lc * */(t[5]    - h/(g+z_acc) * 20.0 * t[3]);
          ineq_ipopt_(var_index(k,dim,B), c) = /* lc * */(t[4]    - h/(g+z_acc) * 12.0 * t[2]);
          ineq_ipopt_(var_index(k,dim,C), c) = /* lc * */(t[3]    - h/(g+z_acc) *  6.0 * t[1]);
          ineq_ipopt_(var_index(k,dim,D), c) = /* lc * */(t[2]    - h/(g+z_acc) *  2.0);
          ineq_ipopt_.col(c)                += /* lc * */ t[1]*Ek;
          ineq_ipopt_.col(c)                += /* lc * */ t[0]*Fk;

        }

        // add the line coefficients
        Eigen::VectorXd line_coefficients_xy = GetXyDimAlternatingVector(l.coeff.p, l.coeff.q);
        ineq.M.col(c) = ineq_ipopt_.col(c).cwiseProduct(line_coefficients_xy);

        ineq_ipopt_vx_[c] = non_dependent_ex*t[0] + non_dependent_fx;
        ineq_ipopt_vy_[c] = non_dependent_ey*t[0] + non_dependent_fy;

        ineq.v[c] += l.coeff.p * ineq_ipopt_vx_[c];
        ineq.v[c] += l.coeff.q * ineq_ipopt_vy_[c];
        ineq.v[c] += l.coeff.r - l.s_margin;
        ++c;
      }
    }
  }

  LOG4CXX_INFO(log_, "Calc. time inequality constraints:\t" << static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC * 1000.0  << "\tms");
  LOG4CXX_DEBUG(log_, "Dim: " << ineq.M.rows() << " x " << ineq.M.cols());
  LOG4CXX_TRACE(log_, "Matrix:\n" << std::setprecision(2) << ineq.M << "\nVector:\n" << ineq.v.transpose());
  return ineq;
}



std::vector<xpp::hyq::SuppTriangle::TrLine>
ZmpOptimizer::LineForConstraint(const SuppTriangles &supp_triangles, double dt) {

  // calculate number of inequality contraints
  std::vector<SuppTriangle::TrLine> line_for_constraint;

  for (const ZmpSpline& s : zmp_splines_)
  {
    if (s.four_leg_supp_) continue; // no constraints in 4ls phase
    int n_nodes =  std::floor(s.duration_/dt);

    SuppTriangle::TrLines3 lines = supp_triangles.at(s.step_).CalcLines();

    for (int i=0; i<n_nodes; ++i) {
      line_for_constraint.push_back(lines.at(0));
      line_for_constraint.push_back(lines.at(1));
      line_for_constraint.push_back(lines.at(2));
    }
  }
  return line_for_constraint;
}


Eigen::VectorXd ZmpOptimizer::GetXyDimAlternatingVector(double x, double y) const
{
  Eigen::VectorXd x_abcd(kOptCoeff);
  x_abcd.fill(x);

  Eigen::VectorXd y_abcd(kOptCoeff);
  y_abcd.fill(y);

  int coeff = zmp_splines_.size() * kOptCoeff * kDim2d;
  Eigen::VectorXd vec(coeff);
  vec.setZero();

  for (const ZmpSpline& s : zmp_splines_) {
    vec.middleRows(var_index(s.id_,X,A), kOptCoeff) = x_abcd;
    vec.middleRows(var_index(s.id_,Y,A), kOptCoeff) = y_abcd;
  }

  return vec;
}


int ZmpOptimizer::var_index(int spline, int dim, int coeff) const {
  return kOptCoeff * kDim2d * spline + kOptCoeff * dim + coeff;
}


ZmpOptimizer::Splines
ZmpOptimizer::AddOptimizedCoefficients(const Position& start_cog_p,
                            const Velocity& start_cog_v,
                            const Eigen::VectorXd& optimized_coeff)
{
  Eigen::VectorXd Ek(optimized_coeff.size());
  Eigen::VectorXd Fk(optimized_coeff.size());
  double non_dependent_e, non_dependent_f;

  for (size_t k=0; k<zmp_splines_.size(); ++k) {
    CoeffValues coeff_values;

    for (int dim=X; dim<=Y; dim++) {

      double* cv = (dim == X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      for (int coeff = 0; coeff < kOptCoeff; ++coeff) {
        cv[coeff] = optimized_coeff[var_index(k,dim,coeff)];
      }

      // calculate e and f coefficients from previous values
      Ek.setZero();
      Fk.setZero();
      DescribeEByPrev(k, dim, start_cog_v(dim), Ek, non_dependent_e);
      DescribeFByPrev(k, dim, start_cog_v(dim), start_cog_p(dim), Fk,non_dependent_f);

      cv[E] = Ek.transpose()*optimized_coeff + non_dependent_e;
      cv[F] = Fk.transpose()*optimized_coeff + non_dependent_f;

    } // dim:X..Y

    zmp_splines_.at(k).set_spline_coeff(coeff_values);

  } // k=0..n_spline_infos_
  return zmp_splines_;
}


void ZmpOptimizer::DescribeEByPrev(int k,
                                   int dim, double start_v, Eigen::VectorXd& Ek,
                                   double& non_dependent) const
{
  Ek.setZero();
  for (int i=0; i<k; ++i) {
    double Ti = zmp_splines_.at(i).duration_;

    Ek(var_index(i,dim,A)) += 5*std::pow(Ti,4);
    Ek(var_index(i,dim,B)) += 4*std::pow(Ti,3);
    Ek(var_index(i,dim,C)) += 3*std::pow(Ti,2);
    Ek(var_index(i,dim,D)) += 2*std::pow(Ti,1);
  }

  non_dependent = start_v;
}


void ZmpOptimizer::DescribeFByPrev(int k,
                                   int dim, double start_v, double start_p,
                                   Eigen::VectorXd& Fk, double& non_dependent) const
{
  Eigen::VectorXd T0tok(k); T0tok.setZero();
  Fk.setZero();
  for (int i=0; i<k; ++i) {

    double Ti = zmp_splines_.at(i).duration_;

    T0tok(i) = Ti; // initial velocity acts over the entire time T of EVERY spline

    Fk[var_index(i,dim,A)] += std::pow(Ti,5);
    Fk[var_index(i,dim,B)] += std::pow(Ti,4);
    Fk[var_index(i,dim,C)] += std::pow(Ti,3);
    Fk[var_index(i,dim,D)] += std::pow(Ti,2);

    for (int j = 0; j<i; ++j) {
      double Tj = zmp_splines_.at(j).duration_;
      Fk[var_index(j,dim,A)] += 5*std::pow(Tj,4) * Ti;
      Fk[var_index(j,dim,B)] += 4*std::pow(Tj,3) * Ti;
      Fk[var_index(j,dim,C)] += 3*std::pow(Tj,2) * Ti;
      Fk[var_index(j,dim,D)] += 2*std::pow(Tj,1) * Ti;
    }
  }

  non_dependent = start_v*T0tok.sum() + start_p;
}


} // namespace zmp
} // namespace xpp
