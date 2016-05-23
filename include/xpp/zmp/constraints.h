/*
 * constraints.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_

#include <Eigen/Dense>

#include <xpp/utils/eigen_num_diff_functor.h>
#include <xpp/zmp/nlp_structure.h>

#include <xpp/zmp/problem_specification.h>
#include <xpp/zmp/spline_constraints.h>
#include <xpp/zmp/zmp_constraint.h>


namespace xpp {
namespace zmp {


class Constraints : public ProblemSpecification,
                    public xpp::utils::EigenNumDiffFunctor<double>{

public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::VectorXd VectorXd;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::zmp::SplineConstraints::State State;

  struct Bound {
    Bound(double lower = 0.0, double upper = 0.0) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };

  enum ConstraintType {
    kFinalState,
    kInitialAcceleration,
    kSmoothJerkAccAtSplineJunctions,
    kZmpInSupport,
    kObstacle,
    kCogToFoothold,
  };



  const Bound kEqualityBound           = Bound(0.0, 0.0);
  const Bound kInequalityBoundPositive = Bound(0.0, 1.0e19);

  const std::map<ConstraintType, Bound> bound_types
  {
    {kZmpInSupport, kInequalityBoundPositive},
    {kObstacle,     kInequalityBoundPositive},

    {kSmoothJerkAccAtSplineJunctions, kEqualityBound},
    {kInitialAcceleration,            kEqualityBound},
    {kFinalState,                     kEqualityBound},

    {kCogToFoothold, Bound(-0.20, 0.20)}
  };


  struct Constraint {
    VectorXd values_;
    ConstraintType type_;
  };


public:
  Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
               const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
               const NlpStructure& nlp_structure,
               double walking_height,
               Vector2d initial_acc,
               State final_state);
  virtual
  ~Constraints () {};


  std::vector<Bound> GetBounds();

  /**
   * This implements the value of the constraints later used by Eigen::NumDiff
   *
   * @param x_coeff the inputs to the function
   * @param obj_value the one-dimensional output (obj_value(0)) of the cost function
   */
  int operator() (const InputType& x, ValueType& obj_value) const
  {
    throw std::runtime_error("Can't differentiate contraints because EvalConstaints() not constant");
//    obj_value = EvalContraints(x);
    return 1;
  }
  Eigen::VectorXd EvalContraints(const InputType& x);
  MatVec spline_junction_constraints_;
  MatVec spline_initial_acc_constraints_;
  MatVec spline_final_constraints_;

  ZmpConstraint zmp_constraint_;

  const double gap_center_x_ = 0.25;
  const double gap_width_x_  = 0.3;
private:

  NlpStructure nlp_structure_;

  // Add constraints here
  Constraint KeepZmpInSuppPolygon(const VectorXd& x_coeff,
                                  const StdVecEigen2d& footholds) const;
  Constraint FixFootholdPosition(const StdVecEigen2d& footholds) const;
  Constraint SmoothAccJerkAtSplineJunctions(const VectorXd& x_coeff) const;
  Constraint InitialAcceleration(const VectorXd& x_coeff) const;
  Constraint FinalState(const VectorXd& x_coeff) const;
  Constraint AddObstacle(const StdVecEigen2d& footholds) const;
  Constraint RestrictFootholdToCogPos(const VectorXd& x_coeff,
                                      const StdVecEigen2d& footholds) const;

  void AppendBounds(int m_constraints, ConstraintType type,
                 std::vector<Bound>& bounds) const;
  std::vector<Constraint> GetConstraintsOnly(const VectorXd& x_coeff,
                                             const StdVecEigen2d& footholds) const;
  VectorXd CombineToEigenVector(const std::vector<Constraint>& g_std) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
