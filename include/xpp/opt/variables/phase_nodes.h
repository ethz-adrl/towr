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

#include <xpp/opt/bound.h>

#include "node_values.h"

namespace xpp {
namespace opt {

class PhaseNodes : public NodeValues {
protected:
  using ContactVector = std::vector<bool>;

  enum Type {Force, Motion};
  PhaseNodes (int n_dim,
              const ContactVector& contact_schedule,
              const std::string& name,
              int n_polys_in_changing_phase,
              Type type);
  ~PhaseNodes();

public:
  virtual void InitializeVariables(const VectorXd& initial_pos,
                                   const VectorXd& final_pos,
                                   const VecDurations& phase_durations) override;

  /** @brief called by contact schedule when variables are updated.
   *
   * Converts phase durations to specific polynomial durations.
   */
  void UpdateDurations(const VecDurations& phase_durations);

private:
  PolyInfoVec BuildPolyInfos(const ContactVector& contact_schedule,
                             int n_polys_in_changing_phase,
                             Type type) const;

  VecDurations ConvertPhaseToSpline(const VecDurations& phase_durations) const;
};


class EneffectorNodes : public PhaseNodes {
public:
  EneffectorNodes (int n_dim,
                   const ContactVector& contact_schedule,
                   const std::string& name,
                   int n_polys_in_changing_phase);
  virtual ~EneffectorNodes();

  virtual VecBound GetBounds() const override;
};


class ForceNodes : public PhaseNodes {
public:
  ForceNodes (int n_dim,
              const ContactVector& contact_schedule,
              const std::string& name,
              int n_polys_in_changing_phase,
              double force_max);
  virtual ~ForceNodes();

  virtual VecBound GetBounds() const override;

private:
  double f_max_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_PHASE_NODES_H_ */
