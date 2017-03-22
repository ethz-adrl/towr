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
  using LoadParams = std::vector<double>;

  EndeffectorLoad ();
  virtual ~EndeffectorLoad ();

  void Init(const EndeffectorsMotion ee_motion, double dt, double T);

  void SetOptimizationVariables(const VectorXd& x);
  VectorXd GetOptimizationVariables() const;
  static constexpr const char* ID = "convexity_lambdas";

  int GetOptVarCount() const;

  LoadParams GetLoadValues(double t) const;

  LoadParams GetLoadValuesIdx(int k) const;

  int GetNumberOfNodes() const;

  int GetNumberOfContacts(int k) const;

  /** @param k the number of discretized node with lambda parameters.
    * @param contact which contacts 0,...,ee we are interested in.
    * @returns the index in the optimization vector where this value is stored
    */
  int Index(int k, int contact) const;




  std::vector<int> GetContactsPerNode() const;

private:
  std::vector<int> n_contacts_per_node_;
  VectorXd lambdas_;
  double dt_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
