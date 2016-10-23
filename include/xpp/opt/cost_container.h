/**
 @file    cost_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to hold all the different cost terms.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_

#include "a_cost.h"
#include "i_observer.h"
#include "optimization_variables.h"
#include <memory>

namespace xpp {
namespace opt {

/** @brief Combines all the cost terms to a cost function and provides value.
  *
  * This class is responsible for knowing about all the different cost terms
  * and delivering the total cost for specific optimization variables.
  */
class CostContainer : public IObserver {
public:
  typedef std::shared_ptr<ACost> CostPtr;
  typedef AConstraint::VectorXd VectorXd;

  CostContainer (OptimizationVariables& subject);
  virtual ~CostContainer ();

  void Update () override;
  void SetWeights(const std::vector<double>&);

  void ClearCosts ();
  void AddCost(CostPtr cost);
  double EvaluateTotalCost () const;
  bool IsEmpty() const;
  VectorXd EvaluateGradient() const;

private:
  std::vector<CostPtr > costs_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_ */
