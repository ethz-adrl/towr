/**
 @file    phase_nodes1.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_PHASE_NODES_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_PHASE_NODES_H_

#include <string>
#include <vector>

#include <ifopt/composite.h>

#include "node_values.h"


namespace xpp {

class PhaseNodes : public NodeValues {
protected:
  using ContactVector = std::vector<bool>;
  using Ptr = std::shared_ptr<PhaseNodes>;

  enum Type {Force, Motion};
  PhaseNodes (int n_dim,
              int phase_count,
              bool is_in_contact_at_start,
              const std::string& name,
              int n_polys_in_changing_phase,
              Type type);
  virtual ~PhaseNodes() = default;

public:
  virtual void InitializeVariables(const VectorXd& initial_pos,
                                   const VectorXd& final_pos,
                                   const VecDurations& phase_durations) override;

  /** @brief called by contact schedule when variables are updated.
   *
   * Converts phase durations to specific polynomial durations.
   */
  void UpdateDurations(const VecDurations& phase_durations);

  Vector3d GetValueAtStartOfPhase(int phase) const;
  int GetNodeIDAtStartOfPhase(int phase) const;

  bool IsConstantPhase(double t) const;

protected:
  bool IsConstantNode(int node_id) const;

  int GetPolyIDAtStartOfPhase(int phase) const;


private:
  PolyInfoVec BuildPolyInfos(int phase_count,
                             bool is_in_contact_at_start,
                             int n_polys_in_changing_phase,
                             Type type) const;

  VecDurations ConvertPhaseToSpline(const VecDurations& phase_durations) const;

  VecDurations phase_durations_; // as opposed to poly_durations in node_values
};


class EEMotionNodes : public PhaseNodes {
public:
  using Ptr = std::shared_ptr<EEMotionNodes>;

  EEMotionNodes (int phase_count,
                 bool is_in_contact_at_start,
                 const std::string& name,
                 int n_polys_in_changing_phase);
  virtual ~EEMotionNodes() = default;

  bool IsContactNode(int node_id) const;

  virtual VecBound GetBounds() const override;
};


class EEForceNodes : public PhaseNodes {
public:
  using Ptr = std::shared_ptr<EEForceNodes>;

  EEForceNodes (int phase_count,
                bool is_in_contact_at_start,
                const std::string& name,
                int n_polys_in_changing_phase);
  virtual ~EEForceNodes() = default;

  bool IsStanceNode(int node_id) const;

  int GetPhase(int node_id) const;

  virtual VecBound GetBounds() const override;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_PHASE_NODES_H_ */
