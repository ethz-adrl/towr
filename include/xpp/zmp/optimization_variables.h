/**
 @file    cost_function_functor.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_

#include <xpp/zmp/a_subject.h>
#include <xpp/zmp/nlp_structure.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

/** @brief hold the state of the optimization variables.
  *
  * This class is responsible for publishing the up-to-date values of the
  * optimization variables to all the observers (cost function,
  * constraints, visualizers,...) that depend on this state.
  *
  * It doesn't know about
  * internal structure of the variables, that is all handled by nlp_structure_.
  * It is able to interpret the values of the optimization variables if they are
  * independent of the initialization values of the NLP.
  *
  * https://sourcemaking.com/design_patterns/observer
  */
class OptimizationVariables : public ASubject {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef NlpStructure::VecBound VecBound;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d; // for footholds
  typedef Eigen::Vector2d Vector2d;

  /** Common strings to use to name optimization variable sets. This is just to
    * avoid misspelling resulting in a set not being found.
    */
  static constexpr const char* kSplineCoeff = "spline_coeff";
  static constexpr const char* kFootholds   = "footholds";

  OptimizationVariables ();
  virtual ~OptimizationVariables ();


  void AddVariableSet(std::string id, const VectorXd& values);

//
//  void Init (int n_spline_coeff, int n_steps);
//  void Init (const VectorXd& x_coeff_abcd, const StdVecEigen2d& footholds);


  VectorXd GetVariables(std::string id) const;
  void SetVariables(std::string id, const VectorXd& values);



  VectorXd GetOptimizationVariables() const;
  VecBound GetOptimizationVariableBounds() const;
//  StdVecEigen2d GetFootholdsStd() const;
//  VectorXd GetSplineCoefficients() const;
  int GetOptimizationVariableCount() const;

  void SetVariables(const VectorXd& x);
//  void SetSplineCoefficients(const VectorXd& x);

  NlpStructure::VariableSetVector GetVarSets() const;

private:
  NlpStructure nlp_structure_; ///< this class holds all the structural information of the NLP
  bool initialized_ = false; // checks if the init() method has been called
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
