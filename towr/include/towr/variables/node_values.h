/**
 @file    node_values.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#ifndef TOWR_VARIABLES_NODE_VALUES_H_
#define TOWR_VARIABLES_NODE_VALUES_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <xpp_states/cartesian_declarations.h>

#include <ifopt/leaves.h>
#include "nodes_observer.h"
#include "polynomial.h"


namespace towr {


/** Holds position and velocity of nodes used to generate a cubic Hermite spline.
 *
 * This however should know nothing about polynomials, or splines or anything.
 * Purely the nodes, everything else handled by spline.h
 */
class NodeValues : public ifopt::VariableSet {
public:
  using Ptr      = std::shared_ptr<NodeValues>;
  using Node     = CubicHermitePoly::Node;
  using Side     = CubicHermitePoly::Side;
  using Deriv    = xpp::MotionDerivative;
  using VecDurations = std::vector<double>;


  NodeValues (int n_dim, const std::string& name);
  NodeValues (int n_dim, int n_nodes, const std::string& name);
  virtual ~NodeValues () = default;



  struct IndexInfo {
    int node_id_;
    Deriv node_deriv_;
    int node_deriv_dim_;

    int operator==(const IndexInfo& right) const;
  };

  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const;

  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const;


  void InitializeNodes(const VectorXd& initial_pos,const VectorXd& final_pos,
                       double t_total);


  VectorXd GetValues () const override;
  void SetVariables (const VectorXd& x) override;






  virtual VecBound GetBounds () const override;

  const std::vector<Node> GetNodes() const;
  int GetPolynomialCount() const;

  // returns the two nodes that make up polynomial with "poly_id"
  const std::vector<Node> GetBoundaryNodes(int poly_id) const;


  static int GetNodeId(int poly_id, Side);
  // basically opposite of above
  int Index(int node_id, Deriv, int dim) const;

  void AddObserver(NodesObserver* const o);

  int GetDim() const;

  void AddStartBound (Deriv d, const std::vector<int>& dimensions, const VectorXd& val);
  void AddFinalBound(Deriv d, const std::vector<int>& dimensions, const VectorXd& val);

protected:
  VecBound bounds_;

  void InitMembers(int n_nodes, int n_variables);

private:
  void UpdateObservers() const;
  std::vector<NodesObserver*> observers_;
  std::vector<Node> nodes_;
  int n_dim_;

  void AddBounds(int node_id, Deriv, const std::vector<int>& dim, const VectorXd& val);
  void AddBound(int node_id, Deriv, int dim, double val);
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
