/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Declares a class to publish the current optimization variables.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_

#include "parametrization.h"

namespace xpp {
namespace opt {

/** @brief Holds and supplies the current value of the optimization variables.
  *
  * This class is responsible for supplying the up-to-date values of the
  * optimization variables to all the observers (cost function,
  * constraints,...) that need these. This class should
  * fit all types of optimization problems and does not have a specific set
  * of variables. The only problem specific information this class holds is the
  * std::string id of what the variables represent.
  */
class OptimizationVariables {
public:
  using VectorXd          = Eigen::VectorXd;
  using VariablePtr       = Parametrization::Ptr;
  using VariableSetVector = std::vector<VariablePtr>;

  OptimizationVariables ();
  virtual ~OptimizationVariables ();

  void ClearVariables();
  void AddVariableSet(const VariablePtr&);
  void SetAllVariables(const VectorXd& x);

  VectorXd GetVariables(std::string id) const;
  VectorXd GetOptimizationVariables() const;
  VecBound GetOptimizationVariableBounds() const;
  int GetOptimizationVariableCount() const;
  VariableSetVector GetVarSets() const;

private:
  VariableSetVector variable_sets_;
  bool SetExists(std::string id) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_ */
