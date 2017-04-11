/**
 @file    cost_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to hold all the different cost terms.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "cost.h"

namespace xpp {
namespace opt {

/** @brief Combines all the cost terms to a cost function and provides value.
  *
  * This class is responsible for knowing about all the different cost terms
  * and delivering the total cost for specific optimization variables.
  */
class CostContainer {
public:
  using VectorXd = Eigen::VectorXd;
  using CostPtr = std::shared_ptr<Cost>;

  CostContainer ();
  virtual ~CostContainer ();

  void SetWeights(const std::vector<double>&);

  void ClearCosts ();
  void AddCost(CostPtr cost, double weight = 1.0);
  double EvaluateTotalCost () const;
  bool IsEmpty() const;
  VectorXd EvaluateGradient() const;

  void UpdateCosts();

private:
  std::vector<CostPtr > costs_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_CONTAINER_H_ */
