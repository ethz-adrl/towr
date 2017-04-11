/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/endeffectors_motion.h>

namespace xpp {
namespace opt {

EndeffectorsMotion::EndeffectorsMotion (const EndeffectorsPos& initial_pos,
                                        const ContactSchedule& contact_schedule)
    :OptimizationVariables("endeffectors_motion")
{
  endeffectors_.SetCount(initial_pos.GetCount());

  SetInitialPos(initial_pos);
  SetParameterStructure(contact_schedule);

  for (const auto& ee : endeffectors_.ToImpl())
    n_opt_params_ += ee.GetOptVarCount();
}

void
EndeffectorsMotion::SetInitialPos (const EndeffectorsPos& initial_pos)
{
  for (auto ee : initial_pos.GetEEsOrdered())
    endeffectors_.At(ee).SetInitialPos(initial_pos.At(ee), ee);
}

void
EndeffectorsMotion::SetParameterStructure (const ContactSchedule& contact_schedule)
{
  for (auto ee : endeffectors_.GetEEsOrdered()) {

    for (auto phase : contact_schedule.GetPhases(ee)) {
      auto is_contact = phase.first;
      auto duration   = phase.second;

      std::cout << "duration: " << duration << std::endl;

      double lift_height = is_contact? 0.0 : 0.03;
      endeffectors_.At(ee).AddPhase(duration, lift_height, is_contact);
    }
  }
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
}

EndeffectorsMotion::EEState
EndeffectorsMotion::GetEndeffectors (double t_global) const
{
  EEState ee_state(GetNumberOfEndeffectors());

  for (auto ee : endeffectors_.GetEEsOrdered())
    ee_state.At(ee) = endeffectors_.At(ee).GetState(t_global);

  return ee_state;
}

EndeffectorsMotion::VectorXd
EndeffectorsMotion::GetVariables () const
{
  VectorXd x(n_opt_params_);

  int row = 0;
  for (const auto& ee : endeffectors_.ToImpl()) {
    int n = ee.GetOptVarCount();
    x.middleRows(row, n) = ee.GetVariables();
    row += n;
  }

  return x;
}

// must be analog to the above
void
EndeffectorsMotion::SetVariables (const VectorXd& x)
{
  int row = 0;

  for (auto ee : endeffectors_.GetEEsOrdered()) {
    int n = endeffectors_.At(ee).GetOptVarCount();
    endeffectors_.At(ee).SetVariables(x.middleRows(row, n));
    row += n;
  }
}

double
EndeffectorsMotion::GetTotalTime () const
{
   return endeffectors_.At(E0).GetTotalTime();
}

int
EndeffectorsMotion::GetNumberOfEndeffectors () const
{
  return endeffectors_.GetCount();
}

JacobianRow
EndeffectorsMotion::GetJacobianWrtOptParams (double t_global,
                                             EndeffectorID ee,
                                             d2::Coords dim) const
{
  JacobianRow jac_row(GetOptVarCount());
  JacobianRow jac_ee = endeffectors_.At(ee).GetJacobianPos(t_global, dim);

  // insert single ee-Jacobian into Jacobian representing all endeffectors
  for (JacobianRow::InnerIterator it(jac_ee); it; ++it)
    jac_row.coeffRef(IndexStart(ee)+it.col()) = it.value();

  return jac_row;
}

int
EndeffectorsMotion::IndexStart (EndeffectorID ee) const
{
  int idx = 0;

  for (int e=E0; e<ee; ++e)
    idx += endeffectors_.At(static_cast<EndeffectorID>(e)).GetOptVarCount();

  return idx;
}

EndeffectorsMotion::EEState::Container
EndeffectorsMotion::GetEndeffectorsVec (double t_global) const
{
  return GetEndeffectors(t_global).ToImpl();
}

} /* namespace opt */
} /* namespace xpp */
