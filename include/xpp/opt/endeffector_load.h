/**
 @file    endeffector_load.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_

#include <xpp/opt/endeffectors_motion.h>

namespace xpp {
namespace opt {

/** Parametrizes the load/force each endeffector is holding during the motion.
  *
  * The are the lambda values in the paper.
  */
class EndeffectorLoad {
public:
  using VectorXd = Eigen::VectorXd;

  EndeffectorLoad ();
  virtual ~EndeffectorLoad ();

  void Init(const EndeffectorsMotion ee_motion, double dt, double T);

  void SetOptimizationVariables(const VectorXd& x);
  VectorXd GetOptimizationVariables() const;
  static constexpr const char* ID = "convexity_lambdas";


  std::vector<int> GetContactsPerNode() const;

private:
  std::vector<int> n_contacts_per_node_;
  VectorXd lambdas_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
