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

  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;


  struct NodeInfo {
    int id_;
    MotionDerivative deriv_;
    int dim_;
  };


  NodeValues ();
  virtual ~NodeValues ();

  void Init(const Node& initial_value, const VecTimes& times, const std::string& name);


  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;

  Jacobian GetJacobian(int poly_id, double t_local, MotionDerivative dxdt) const;
  VecPoly GetCubicPolys() const { return cubic_polys_; };

  VecTimes GetDurations() const { return timings_; };

protected:
  std::vector<NodeInfo> GetNodeInfo(int idx) const;
  std::vector<Node> nodes_;
  int n_dim_;

  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;
  std::map<OptNodeIs, NodeIds > opt_to_spline_; // lookup

private:
  void UpdatePolynomials();
  int GetNodeId(int poly_id, Side) const;

  VecTimes timings_; // zmp_ for now constant
  VecPoly cubic_polys_;
};

class PhaseNodes : public NodeValues {
public:
  PhaseNodes (const Node& initial_value,
              const VecTimes& phase_times,
              const std::string& name,
              bool is_first_phase_constant,
              int n_polys_in_changing_phase);
  ~PhaseNodes();
};



class EEMotionNodes : public PhaseNodes {
public:
  EEMotionNodes (const Node& initial_position, const VecTimes& phase_times,
                 int splines_per_swing_phase, int ee_id);
  ~EEMotionNodes();
  VecBound GetBounds () const override;
};


class EEForcesNodes : public PhaseNodes {
public:
  EEForcesNodes (const Node& initial_force, const VecTimes& phase_times,
                 int splines_per_stance_phase, int ee_id);
  ~EEForcesNodes();
  VecBound GetBounds () const override;
};




// zmp_ might be able to remove this class
class HermiteSpline : public Spline {
public:
  using NodeValueT = std::shared_ptr<NodeValues>;


  HermiteSpline(const OptVarsPtr& opt_vars, const std::string& spline_base_id);
  virtual ~HermiteSpline();

  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  Jacobian GetJacobian(double t_global,  MotionDerivative dxdt) const override;

private:
  virtual VecTimes GetDurations() const override { return node_values_->GetDurations(); };
  NodeValueT node_values_;
};




} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
