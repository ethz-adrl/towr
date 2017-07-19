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

#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

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


  NodeValues (const VecTimes& times, const Node& initial_value,
              const std::string& id);
  virtual ~NodeValues ();

  /**
   * ordered (x0,  y0,  z0,
   *          xd0, yd0, zd0,
   *          x1,  y1,  z1,
   *          xd1, yd1, zd1,...
   */
  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;
  Jacobian GetJacobianOfPosWrtNodes(double t_global) const;

  int Index(int node, MotionDerivative deriv, int dim) const;
  int GetNodeId(int poly_id, Side) const;


private:
  // zmp_ DRY with "Spline"...
  VecTimes durations_; ///< duration of each polynomial in spline
  std::vector<Node> nodes_;
  int n_dim_;

  std::vector<CubicHermitePoly> polynomials_;

  void UpdatePolynomials();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
