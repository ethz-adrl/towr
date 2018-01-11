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


  struct PolyInfo {
    PolyInfo() {};
    PolyInfo(int phase, int poly_id_in_phase,
             int num_polys_in_phase, bool is_constant)
        :phase_(phase), poly_id_in_phase_(poly_id_in_phase),
         num_polys_in_phase_(num_polys_in_phase), is_constant_(is_constant) {}

    int phase_;
    int poly_id_in_phase_;
    int num_polys_in_phase_;
    bool is_constant_; // zmp_ this shouldn't be here, has to do with phases
  };

  using PolyInfoVec = std::vector<PolyInfo>;

  struct NodeInfo {

    int id_; // the actual id of the node, not the optimized node
    MotionDerivative deriv_;
    int dim_;

    // for use with std map
    bool operator <( const NodeInfo &rhs ) const
    {
       if         (id_ != rhs.id_)    return    id_ < rhs.id_;
       else if (deriv_ != rhs.deriv_) return deriv_ < rhs.deriv_;
       else                           return dim_   < rhs.dim_;
    }
  };



  NodeValues (int n_dim, int n_polynomials, const std::string& name);
  NodeValues (int n_dim, const PolyInfoVec&, const std::string& name);
  virtual ~NodeValues () = default;


  virtual void InitializeVariables(const VectorXd& initial_pos,
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


  virtual std::vector<NodeInfo> GetNodeInfoAtOptIndex(int idx) const;
  int GetNodeId(int poly_id, Side) const;
  // basically opposite of above
  int Index(int node_id, MotionDerivative, int dim) const;

  void AddObserver(NodesObserver* const o);

  int n_dim_;

  // smell this should be private
  PolyInfoVec polynomial_info_;
protected:

  std::vector<int> GetAdjacentPolyIds(int node_id) const;

  mutable VecBound bounds_;
  std::vector<Node> nodes_;


  void CacheNodeInfoToIndexMappings();
private:
  PolyInfoVec BuildPolyInfos(int num_polys) const;

  void UpdateObservers() const;
  std::vector<NodesObserver*> observers_;


//  void SetNodeMappings();
//  using OptNodeIs = int;
//  using NodeIds   = std::vector<int>;
//
//  // this could be removed i feel like
//  // maps from the nodes that are actually optimized over
//  // to all the nodes. Optimized nodes are sometimes used
//  // twice in a constant phase.
//  std::map<OptNodeIs, NodeIds > optnode_to_node_; // lookup


  std::map<NodeInfo, int> node_info_to_idx;


};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
