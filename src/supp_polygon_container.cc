/*
 * supp_triangle_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace hyq {

SupportPolygonContainer::SupportPolygonContainer ()
{
  // TODO Auto-generated constructor stub

}

SupportPolygonContainer::~SupportPolygonContainer ()
{
  // TODO Auto-generated destructor stub
}


void SupportPolygonContainer::Init(LegDataMap<Foothold> start_stance,
                                 const VecFoothold& footholds,
                                 const MarginValues& margins)
{
  start_stance_ = start_stance;
  footholds_    = footholds;
  margins_      = margins;

  initialized_ = true;
}


std::vector<SupportPolygonContainer::SplineAndSupport>
SupportPolygonContainer::CombineSplineAndSupport(const VecFoothold& footholds,
                                            const xpp::zmp::ContinuousSplineContainer& zmp_splines) const
{
  CheckIfInitialized();

  std::vector<SplineAndSupport> spline_and_support(zmp_splines.splines_.size());

  LegDataMap<Foothold> curr_stance = start_stance_;
  for (const xpp::zmp::ZmpSpline& s : zmp_splines.splines_)
  {
    spline_and_support.at(s.id_).spline_ = s;
    VecFoothold legs_in_contact;
    const Foothold& f = footholds.at(s.step_);
    int swingleg = f.leg;

    // extract the 3 non-swinglegs from stance or use all four if four-leg-support
    for (LegID l : LegIDArray)
      if(curr_stance[l].leg != swingleg || s.four_leg_supp_)
        legs_in_contact.push_back(curr_stance[l]);

    spline_and_support.at(s.id_).support_ = SupportPolygon(margins_, legs_in_contact);

    // update current stance with last step if leg just swung
    if (!s.four_leg_supp_)
      curr_stance[swingleg] = f;
  }
  return spline_and_support;
}


Eigen::Vector2d
SupportPolygonContainer::GetCenterOfFinalStance() const
{
  CheckIfInitialized();

  LegDataMap<Foothold> last_stance = start_stance_;

  // get the last step of each foot
  Foothold f;
  for (LegID l : LegIDArray)
    if(Foothold::GetLastFoothold(l,footholds_, f))
      last_stance[f.leg] = f;

  // calculate average x-y-postion of last stance
  Eigen::Vector2d end_cog = Eigen::Vector2d::Zero();
  for (LegID l : LegIDArray)
    end_cog += last_stance[l].p.segment<2>(xpp::utils::X);

  return end_cog/_LEGS_COUNT;
}


SupportPolygonContainer::MatVec
SupportPolygonContainer::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                            const xpp::zmp::ContinuousSplineContainer& zmp_splines) const
{
  CheckIfInitialized();

  int coeff = zmp_splines.GetTotalFreeCoeff();

  int num_nodes_no_4ls = zmp_splines.GetTotalNodesNo4ls();
  int num_nodes_4ls = zmp_splines.GetTotalNodes4ls();
  int num_ineq_constr = 3*num_nodes_no_4ls + 4*num_nodes_4ls;

  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter
  for (const SplineAndSupport& s : CombineSplineAndSupport(zmp_splines)) {

    SupportPolygon::VecSuppLine lines = s.support_.CalcLines();
    for (double i=0; i < s.spline_.GetNodeCount(zmp_splines.dt_); ++i) {

      // add three line constraints for each node
      for (SupportPolygon::SuppLine l : lines) {
        AddLineConstraint(l, x_zmp.M.row(n), y_zmp.M.row(n),
                             x_zmp.v[n],     y_zmp.v[n],
                             c, ineq);
      }
      n++;
    }
  }
  return ineq;
}


void SupportPolygonContainer::AddLineConstraint(const SupportPolygon::SuppLine& l,
                                    const Eigen::RowVectorXd& x_zmp_M,
                                    const Eigen::RowVectorXd& y_zmp_M,
                                    double x_zmp_v,
                                    double y_zmp_v,
                                    int& c, MatVec& ineq) const
{
  // the zero moment point must always lay on one side of triangle side:
  // p*x_zmp + q*y_zmp + r > stability_margin
  ineq.M.row(c) = l.coeff.p*x_zmp_M + l.coeff.q*y_zmp_M;
  ineq.v[c]     = l.coeff.p*x_zmp_v + l.coeff.q*x_zmp_v;
  ineq.v[c]    += l.coeff.r - l.s_margin;

  c++;
}


bool SupportPolygonContainer::Insert4LSPhase(LegID prev, LegID next)
{
  using namespace xpp::hyq;
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}


void
SupportPolygonContainer::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("SuppTriangleContainer not initialized. Call Init() first");
  }
}




} /* namespace hyq */
} /* namespace xpp */
