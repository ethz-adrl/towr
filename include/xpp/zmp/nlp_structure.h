/**
 @file    nlp_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Declares the class NlpStructure
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace xpp {
namespace zmp {

class VariableSet;

/** @brief Holds the optimization variables.
  *
  * This class is responsible for holding the current values of the optimization
  * variables and providing a name what each variable represents. It
  * returns only exactly those values.
  */
class NlpStructure {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;
  typedef std::unique_ptr<VariableSet> VariableSetPtr;
  typedef std::vector<VariableSetPtr> VariableSetVector;

  NlpStructure();
  virtual ~NlpStructure();

  /** A variable set is a block of variables with some semantic information.
    *
    * @param name What the variables represents.
    * @param n_variables The number of variables.
    */
  void AddVariableSet(std::string name, int n_variables);


  VectorXd GetOptimizationVariables() const;
  int GetOptimizationVariableCount() const;

  void SetAllVariables(const VectorXd& x_all);
  void SetAllVariables(const Number* x_all);

  void SetVariables(std::string set_name, const VectorXd& values);
  VectorXd GetVariables(std::string set_name) const;

  void Reset();

private:
  VariableSetVector variable_sets_;
  int n_variables_;
  VectorXd ConvertToEigen(const Number* x) const;
};


} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
