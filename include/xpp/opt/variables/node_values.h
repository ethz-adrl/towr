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

#include "contact_schedule.h"
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




  struct NodeInfo {
    int id_;
    MotionDerivative deriv_;
    int dim_;
  };


  NodeValues ();
  virtual ~NodeValues ();

  void Init(const Node& initial_value, int n_polynomials, const std::string& name);


  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;



  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  virtual const StateLinXd GetPoint(double t_global) const override;
  virtual Jacobian GetJacobian (double t_global,  MotionDerivative dxdt) const override;





protected:
  std::vector<NodeInfo> GetNodeInfo(int idx) const;
  std::vector<Node> nodes_;
  int n_dim_;

  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;
  std::map<OptNodeIs, NodeIds > opt_to_spline_; // lookup

  mutable VecPoly cubic_polys_;
  void UpdatePolynomials() const;
  int GetNodeId(int poly_id, Side) const;

  VecTimes times_;
private:

  virtual void SetNodeMappings();

  Jacobian GetJacobian(int poly_id, double t_local, MotionDerivative dxdt) const;



};

class PhaseNodes : public NodeValues {
public:
  using SchedulePtr = std::shared_ptr<ContactSchedule>;

  PhaseNodes (const Node& initial_value,
              const SchedulePtr& contact_schedule,
              const std::string& name,
              bool is_constant_during_contact,
              int n_polys_in_changing_phase);
  ~PhaseNodes();


  VectorXd GetDerivativeOfPosWrtPhaseDuration(double t_global) const;

  void UpdateTimes(const VecTimes& durations);

private:



  void SetTimeStructure(int num_phases,
                        bool is_constant_during_contact,
                        const SchedulePtr& contact_schedule,
                        int polys_in_non_constant_phase);

  virtual void SetNodeMappings () override;



  using PhaseID = int;
  using LocalID = int;
  using LocalCount = int;
  using IsContant = bool;
  using DurationInfo = std::tuple<PhaseID, LocalID, LocalCount, IsContant>;
  // zmp_ rename this
  mutable std::vector<DurationInfo> percent_of_phase_;
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
