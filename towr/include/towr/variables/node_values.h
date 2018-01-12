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
#include <xpp_states/state.h>

#include <ifopt/leaves.h>
#include "nodes_observer.h"
#include "polynomial.h" // this shouldn't really be here



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
  using VecNodes = std::vector<Node>;
  using MotionDerivative = xpp::MotionDerivative;
  using VecDurations = std::vector<double>;


  // smell remove this c'tor
  // b/c doesn't fully initialize object
  NodeValues (int n_dim, const std::string& name);
  NodeValues (int n_dim, int n_nodes, const std::string& name);
  virtual ~NodeValues () = default;


  // ------ virtual functions -------------
  struct IndexInfo {
    int node_id_; // the actual id of the node, not the optimized node
    MotionDerivative deriv_;
    int dim_;

    int operator==(const IndexInfo& right) const;
  };

  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const;

  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const;


  virtual void InitializeNodes(const VectorXd& initial_pos,
                                   const VectorXd& final_pos,
                                   double t_total);


  VectorXd GetValues () const override;
  void SetVariables (const VectorXd& x) override;



  void AddBounds(int node_id, MotionDerivative, const std::vector<int>& dim,
                 const VectorXd& val);
  void AddBound(int node_id, MotionDerivative, int dim, double val);
  void AddStartBound (MotionDerivative d,
                      const std::vector<int>& dimensions,
                      const VectorXd& val);
  void AddFinalBound(MotionDerivative d,
                     const std::vector<int>& dimensions,
                     const VectorXd& val);



  virtual VecBound GetBounds () const override { return bounds_;};

  const std::vector<Node> GetNodes() const { return nodes_; };
  int GetPolynomialCount() const { return nodes_.size() - 1; };

  // returns the two nodes that make up polynomial with "poly_id"
  const std::vector<Node> GetBoundaryNodes(int poly_id) const;


  int GetNodeId(int poly_id, Side) const;
  // basically opposite of above
  int Index(int node_id, MotionDerivative, int dim) const;

  void AddObserver(NodesObserver* const o);

  int GetDim() const { return n_dim_; };


protected:
  int n_dim_;
  mutable VecBound bounds_;
  std::vector<Node> nodes_;

private:
  void UpdateObservers() const;
  std::vector<NodesObserver*> observers_;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
