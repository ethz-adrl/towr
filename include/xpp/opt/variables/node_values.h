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
#include <tuple>
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
  using VecTimes = std::vector<double>;

  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;




  using PhaseID = int;
  using LocalID = int;
  using LocalCount = int;
  using IsContant = bool;
  using DurationInfo = std::tuple<PhaseID, LocalID, LocalCount, IsContant>;
  using PolyInfoVec = std::vector<DurationInfo>;




  struct NodeInfo {
    int id_;
    MotionDerivative deriv_;
    int dim_;
  };


  NodeValues ();
  virtual ~NodeValues ();

//  void Init(const Node& initial_value, int n_polys, const std::string& name);
  void Init(const Node& initial_value, const PolyInfoVec&, const std::string& name);


  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;



  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  virtual const StateLinXd GetPoint(double t_global) const override;
  virtual Jacobian GetJacobian (double t_global,  MotionDerivative dxdt) const override;


  VectorXd GetDerivativeOfPosWrtPhaseDuration(double t_global) const;


protected:
  std::vector<NodeInfo> GetNodeInfo(int idx) const;

  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;
  std::map<OptNodeIs, NodeIds > opt_to_spline_; // lookup

  void UpdatePolynomials();
  int GetNodeId(int poly_id, Side) const;
  VecTimes times_;




  PolyInfoVec polynomial_info_;


private:
  std::vector<Node> nodes_;
  int n_dim_;
  VecPoly cubic_polys_;
  virtual void SetNodeMappings();

  Jacobian GetJacobian(int poly_id, double t_local, MotionDerivative dxdt) const;
};


class PhaseNodes : public NodeValues {
public:
  using SchedulePtr = std::vector<bool>;

  PhaseNodes (const Node& initial_value,
              const SchedulePtr& contact_schedule,
              const std::string& name,
              bool is_constant_during_contact,
              int n_polys_in_changing_phase);
  ~PhaseNodes();


  /** @brief called by contact schedule when variables are updated.
   *
   * Converts phase durations to specific polynomial durations.
   */
  void UpdateTimes(const VecTimes& phase_durations);

private:


//  virtual void SetNodeMappings () override;



};



class EEMotionNodes : public PhaseNodes {
public:
  EEMotionNodes (const Node& initial_position,
                 const SchedulePtr& contact_schedule,
                 int splines_per_swing_phase,
                 int ee_id);
  ~EEMotionNodes();
  VecBound GetBounds () const override;
};


class EEForcesNodes : public PhaseNodes {
public:
  EEForcesNodes (const Node& initial_force,
                 const SchedulePtr& contact_schedule,
                 int splines_per_stance_phase,
                 int ee_id);
  ~EEForcesNodes();
  VecBound GetBounds () const override;
};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
