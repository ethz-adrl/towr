/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Declares a class to publish the current optimization variables.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_

#include "subject.h"
#include "variable_set.h"

namespace xpp {
namespace opt {


/** @brief Holds and publishes the current value of the optimization variables.
  *
  * This class is responsible for publishing the up-to-date values of the
  * optimization variables to all the observers (cost function,
  * constraints,...) that registered to this. This class should
  * fit all types of optimization problems and does not have a specific set
  * of variables. The only problem specific information this class holds is the
  * std::string id of what the variables represent.
  *
  * https://sourcemaking.com/design_patterns/observer
  */
class OptimizationVariables : public Subject {
public:
  using VectorXd = Eigen::VectorXd;
  using VariableSetVector = std::vector<VariableSet>;

  OptimizationVariables ();
  virtual ~OptimizationVariables ();

  void ClearVariables();
  void AddVariableSet(const VariableSet&);
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
