/**
 @file    node_values.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_

#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

#include "spline.h"

namespace xpp {
namespace opt {

/** Holds position and velocity of nodes used to generate a cubic Hermite spline.
 */
class NodeValues : public Component {
public:
  using Node     = CubicHermitePoly::Node;
  using Side     = CubicHermitePoly::Side;
  using VecNodes = std::vector<Node>;
  using VecTimes = std::vector<double>;


//  using OptVarsPtr = Primitive::OptVarsPtr;
  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;


  NodeValues (const Node& initial_value, const VecTimes&, const std::string& name);
  virtual ~NodeValues ();

  /**
   * ordered (x0,  y0,  z0,
   *          xd0, yd0, zd0,
   *          x1,  y1,  z1,
   *          xd1, yd1, zd1,...
   */
  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;

//  const StateLinXd GetPoint(double t_global) const;



//  VarsPtr GetActiveVariableSet(double t_global) const = 0;

  Jacobian GetJacobian(int poly_id, double t_local, double T) const;



  VecPoly GetCubicPolys() const { return cubic_polys_; };


private:
  int Index(int node, MotionDerivative deriv, int dim) const;
  int GetNodeId(int poly_id, Side) const;

  // zmp_ DRY with "Spline"...
  std::vector<Node> nodes_;
  int n_dim_;

  VecPoly cubic_polys_;
  VecTimes timings_; // zmp_ for now constant, remove at some point
  void UpdatePolynomials(const VecTimes& durations);

};




class HermiteSpline : public Spline {
public:
  using NodeValueT = std::shared_ptr<NodeValues>;

  // factory method
  static Spline::Ptr BuildSpline(const OptVarsPtr& opt_vars,
                                 const std::string& spline_base_id,
                                 const VecTimes& poly_durations);


  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  Jacobian GetJacobian(double t_global,  MotionDerivative dxdt) const override;


  void SetNodeValues(NodeValueT opt)
  {
    node_values_ = opt;
    auto v = opt->GetCubicPolys();
    polynomials_.assign(v.begin(), v.end()); // zmp_ links the two?
  };

private:
  NodeValueT node_values_;
};




} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
