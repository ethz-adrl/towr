/**
 @file    node_values.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>
#include <xpp/state.h>

#include "spline.h"

namespace xpp {
namespace opt {


/** Holds position and velocity of nodes used to generate a cubic Hermite spline.
 */
class NodeValues : public Component, public Spline {
public:
  using Node     = CubicHermitePoly::Node;
  using Side     = CubicHermitePoly::Side;
  using VecNodes = std::vector<Node>;
  using VecDurations = std::vector<double>;

  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;


  struct PolyInfo {
    PolyInfo() {};
    PolyInfo(int phase, int poly_id_in_phase,
             int num_polys_in_phase, bool is_constant)
        :phase_(phase), poly_id_in_phase_(poly_id_in_phase),
         num_polys_in_phase_(num_polys_in_phase), is_constant_(is_constant) {}

    int phase_;
    int poly_id_in_phase_;
    int num_polys_in_phase_;
    bool is_constant_;
  };

  using PolyInfoVec = std::vector<PolyInfo>;

  struct NodeInfo {
    int id_;
    MotionDerivative deriv_;
    int dim_;
  };

  NodeValues (int n_dim, int n_polynomials, const std::string& name);
  NodeValues (int n_dim, const PolyInfoVec&, const std::string& name);
  virtual ~NodeValues ();


  virtual void InitializeVariables(const VectorXd& initial_pos,
                                   const VectorXd& final_pos,
                                   const VecDurations& poly_durations);


  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;
  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  virtual const StateLinXd GetPoint(double t_global) const override;
  virtual Jacobian GetJacobian (double t_global,  MotionDerivative dxdt) const override;

  VectorXd GetDerivativeOfPosWrtPhaseDuration(double t_global) const;

  void AddBound(int node_id, MotionDerivative, int dim, double val);
  void AddStartBound (MotionDerivative d, const VectorXd& val);
  void AddIntermediateBound (MotionDerivative d, const VectorXd& val);
  void AddFinalBound(MotionDerivative, const VectorXd& val);


  virtual VecBound GetBounds () const override { return bounds_;};



  VectorXd GetPositionValues() const;

  const std::vector<Node> GetNodes() const { return nodes_; };

  std::vector<NodeInfo> GetNodeInfo(int idx) const;
  int GetDimCount() const { return n_dim_; };

protected:
  void UpdatePolynomials();
  bool durations_change_ = false;
  VecDurations poly_durations_;
  PolyInfoVec polynomial_info_;
  VecBound bounds_;


private:
  PolyInfoVec BuildPolyInfos(int num_polys) const;

  int n_dim_;
  VecPoly cubic_polys_;
  std::vector<Node> nodes_;

  int GetNodeId(int poly_id, Side) const;
  void SetNodeMappings();
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;
  std::map<OptNodeIs, NodeIds > opt_to_spline_; // lookup

  void FillJacobian(int poly_id, double t_local, MotionDerivative dxdt,
                    Jacobian& jac) const;

};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
