/**
 @file    optimization_variables_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Declares a class to publish the current optimization variables.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_CONTAINER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_CONTAINER_H_

#include <string>
#include <vector>
#include <Eigen/Dense>

#include "bound.h"
#include "optimization_variables.h"

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
// spring_clean_ should also be a component in a composite design pattern,
// just like optimization_variables.
// see https://sourcemaking.com/design_patterns/composite
class OptimizationVariablesContainer {
public:
  using VectorXd   = Eigen::VectorXd;
  using OptVarsPtr = OptimizationVariables::Ptr;
  using OptVarsVec = std::vector<OptVarsPtr>;

  OptimizationVariablesContainer ();
  virtual ~OptimizationVariablesContainer ();

  // component specific functions
  VectorXd GetValues() const;
  VecBound GetBounds() const;
  int GetRows() const;

  // optimization specific functions
  void SetValues(const VectorXd& x);

  // composite specific functions
  void AddComponent(const OptVarsPtr&);
  void ClearComponents();
  OptVarsVec GetComponents() const;
  OptVarsPtr GetComponent(std::string id) const;

private:
  OptVarsVec components_;
//  bool SetExists(std::string id) const;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_CONTAINER_H_ */
