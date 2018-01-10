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
#include "node_observer.h"

//#include "spline.h"
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
  using VecDurations = std::vector<double>;
  using MotionDerivative = xpp::MotionDerivative;

  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;
  using Dimensions = std::vector<xpp::Coords3D>;

//  mutable bool fill_jacobian_structure_ = true;
//  mutable Jacobian jac_structure_; // all zeros

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
                                   const VecDurations& poly_durations);


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
  const std::vector<Node> GetBoundaryNodes(int poly_id) const
  {
    std::vector<Node> nodes;
    nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::Start)));
    nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::End)));
    return nodes;
  };


  std::vector<NodeInfo> GetNodeInfo(int idx) const;
  int GetNodeId(int poly_id, Side) const;
  // basically opposite of above
  int Index(int node_id, MotionDerivative, int dim) const;


  void AddObserver(const NodeObserver::Ptr& o)
  {
    observers_.push_back(o);
    UpdateObservers();
  };

  int n_dim_;

  PolyInfoVec polynomial_info_;
protected:
//  void UpdatePolynomials();
//  bool durations_change_ = false;
//  VecDurations poly_durations_;

  std::vector<int> GetAdjacentPolyIds(int node_id) const;

  mutable VecBound bounds_;

  std::vector<Node> nodes_;


private:
  PolyInfoVec BuildPolyInfos(int num_polys) const;
//  VecPoly cubic_polys_;

  void UpdateObservers() const;
  std::vector<NodeObserver::Ptr> observers_;


  void SetNodeMappings();
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;

  // this could be removed i feel like
  std::map<OptNodeIs, NodeIds > opt_to_spline_; // lookup

  // fill_with_zeros is to get sparsity
//  void FillJacobian(int poly_id, double t_local, MotionDerivative dxdt,
//                    Jacobian& jac, bool fill_with_zeros=false) const;


  std::map<NodeInfo, int> node_info_to_idx;
  void SetIndexMappings();

//  void SetBoundsAboveGround();

};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
