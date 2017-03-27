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
  using LoadParams = Endeffectors<double>;//std::vector<double>;

  EndeffectorLoad ();
  virtual ~EndeffectorLoad ();

  void Init(const EndeffectorsMotion ee_motion, double dt, double T);

  void SetOptimizationVariables(const VectorXd& x);
  VectorXd GetOptimizationVariables() const;
  static constexpr const char* ID = "convexity_lambdas";


  int GetOptVarCount() const;

  LoadParams GetLoadValues(double t) const;

  LoadParams GetLoadValuesIdx(int k) const;


  int GetNumberOfSegments() const;
//  int GetNumberOfContacts(int k) const;

  /** @param k the number of discretized node with lambda parameters.
    * @param contact which contacts 0,...,ee we are interested in.
    * @returns the index in the optimization vector where this value is stored
    */
  // zmp_ this is too error prone having two functions, remove one
  int IndexDiscrete(int k, EndeffectorID ee) const;
  int Index(double t, EndeffectorID ee) const;

  double GetTStart(int node) const;



  std::vector<int> GetContactsPerNode() const;

private:
//  VectorXd lambdas_; // spring_clean_ remove
//  std::vector<int> n_contacts_per_node_;

  int n_ee_; ///< number of endeffectors
  VectorXd lambdas_new_;
  double dt_;

  int GetSegment(double t) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
