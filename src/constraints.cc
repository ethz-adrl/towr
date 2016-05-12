/*
 * constraints.cc
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraints.h>
#include <xpp/zmp/spline_constraints.h>

#include <Eigen/Sparse>

#define prt(x) std::cout << #x << " = " << x << std::endl;
//#define prt(x)

namespace xpp {
namespace zmp {

Constraints::Constraints (const xpp::hyq::SupportPolygonContainer& supp_poly_container,
                          const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
                          const NlpStructure& nlp_structure,
                          double walking_height,
                          Vector2d initial_acc,
                          State final_state)
    :ProblemSpecification(supp_poly_container, zmp_spline_container),
     xpp::utils::EigenNumDiffFunctor<double>(nlp_structure.GetOptimizationVariableCount(),1),
     zmp_constraint_(zmp_spline_container, walking_height)
{

  nlp_structure_ = nlp_structure;

  SplineConstraints spline_constraint(zmp_spline_container);
  spline_junction_constraints_    = spline_constraint.CreateJunctionConstraints();
  spline_initial_acc_constraints_ = spline_constraint.InitialAccJerkConstraints(initial_acc);
  spline_final_constraints_       = spline_constraint.CreateFinalConstraints(final_state);

  // initializes number of constraints
  Constraints::GetBounds();
}


Constraints::VectorXd
Constraints::EvalContraints(const InputType& x)
{
  VectorXd x_coeff = nlp_structure_.ExtractSplineCoefficients(x);
  StdVecEigen2d footholds = nlp_structure_.ExtractFootholds(x);

  // IMPORTANT: update trajectory and footholds first
  UpdateCurrentState(x_coeff, footholds);

  std::vector<Constraint> g_std = GetConstraintsOnly(x_coeff, footholds);

  return CombineToEigenVector(g_std);
}


std::vector<Constraints::Constraint>
Constraints::GetConstraintsOnly(const VectorXd& x_coeff,
                                const StdVecEigen2d& footholds) const
{
  std::vector<Constraint> g_std;

  // generate constraint violation values
  // ATTENTION: order seems to play a role
  g_std.push_back(FinalState(x_coeff));
  g_std.push_back(InitialAcceleration(x_coeff));
  g_std.push_back(SmoothAccJerkAtSplineJunctions(x_coeff));
  g_std.push_back(KeepZmpInSuppPolygon(x_coeff, footholds));
  g_std.push_back(RestrictFootholdToCogPos(x_coeff, footholds));
//  g_std.push_back(AddObstacle(footholds));
//  g_std.push_back(FixFootholdPosition(footholds));


  return g_std;
}



Constraints::Constraint
Constraints::KeepZmpInSuppPolygon(const VectorXd& x_coeff,
                                  const StdVecEigen2d& footholds) const
{
  MatVec ineq = zmp_constraint_.CalcZmpConstraints(supp_polygon_container_);

  Constraint constraints;
  constraints.values_ = ineq.M*x_coeff + ineq.v;
  constraints.type_ = kZmpInSupport;

  return constraints;
}


Constraints::Constraint
Constraints::AddObstacle(const StdVecEigen2d& footholds) const
{
  Constraint constraints;
  constraints.values_ = DistanceSquareFootToGapboarder(footholds, gap_center_x_, gap_width_x_);
  constraints.type_ = kObstacle;
  return constraints;
}


Constraints::Constraint
Constraints::RestrictFootholdToCogPos(const VectorXd& x_coeff,
                                      const StdVecEigen2d& footholds) const
{
  Constraint c;
  c.values_ = DistanceFootToNominalStance(x_coeff, footholds);
  c.type_ = kCogToFoothold;
  return c;
}


Constraints::Constraint
Constraints::SmoothAccJerkAtSplineJunctions(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_junction_constraints_.M*x_coeff + spline_junction_constraints_.v;
  c.type_ = kSmoothJerkAccAtSplineJunctions;
  return c;
}


Constraints::Constraint
Constraints::InitialAcceleration(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_initial_acc_constraints_.M*x_coeff + spline_initial_acc_constraints_.v;
  c.type_ = kInitialAcceleration;
  return c;
}


Constraints::Constraint
Constraints::FinalState(const VectorXd& x_coeff) const
{
  Constraint c;
  c.values_ = spline_final_constraints_.M*x_coeff + spline_final_constraints_.v;
  c.type_ = kFinalState;
  return c;
}


//Constraints::Constraint
//Constraints::FixFootholdPosition(const StdVecEigen2d& footholds) const
//{
//  Constraint constraints;
//  constraints.values_ = DistanceFootFromPlanned(footholds);
//  constraints.type_ = EQUALITY;
//
//  return constraints;
//}


std::vector<Constraints::Bound>
Constraints::GetBounds()
{
  std::vector<Constraints::Bound> bounds;

  // non initialized values just to compute the bounds
  Eigen::VectorXd x_coeff(zmp_spline_container_.GetTotalFreeCoeff());
  StdVecEigen2d x_footholds(supp_polygon_container_.GetNumberOfSteps());
  x_coeff.setZero();
  for (Vector2d& f : x_footholds)
    f.setZero();


  std::vector<Constraint> g_std = GetConstraintsOnly(x_coeff, x_footholds);

  for (const Constraint& g : g_std) {
    AppendBounds(g.values_.rows(), g.type_, bounds);
  }

  m_values = bounds.size();
  return bounds;
}



//std::vector<Eigen::Triplet<double> >
//Constraints::GetSparsityStructureInitialAcc(const Constraint& g) const
//{
//  std::vector<Eigen::Triplet<double> > nonzero_cell;
//
//  // go through every constraint
//  for (int row=0; row < g.values_.rows(); ++row) {
//    // decide which optimization variable affects this particular constraint
//
//
//  }
//
//
//  return nonzero_cell;
//}





void
Constraints::AppendBounds(int m_constraints, ConstraintType type,
                       std::vector<Constraints::Bound>& bounds) const
{
  for (int c=0; c<m_constraints; ++c) {
    bounds.push_back(bound_types.at(type));
  }
}


Constraints::VectorXd
Constraints::CombineToEigenVector(const std::vector<Constraint>& g_std) const
{
  VectorXd g_eig(m_values);
  //  combine all the g vectors
  //  g_ << g_vec[0], g_vec[1], g_vec[2];
  int c = 0;
  for (const Constraint& g : g_std) {
    g_eig.middleRows(c, g.values_.rows()) = g.values_; //g.normalized()
    c += g.values_.rows();
  }
  return g_eig;
}


} /* namespace zmp */
} /* namespace xpp */
