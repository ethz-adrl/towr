/**
@file   hyq_state-inl.h
@author Alexander Winkler (winklera@ethz.ch)
@date   Jul 20, 2014
@brief	Inline function definitions included in the header file hyq_state.h.
*/

inline void HyqState::RampInPos(double des, Coords3D coord,
                                    double t, double t_max)
{
  if (t > t_max)
    base_.pos.p(coord) = des;

  else {
    double p = 1.0 - 1.0/t_max * t; // p(t=0) = 1 -->  p(t=t_max) = 0
    base_.pos.p(coord) = (p * base_.pos.p(coord) + (1-p) * des);
  }
}

inline int HyqState::SwinglegID() const
{
  for (LegID leg : LegIDArray)
    if (swingleg_[leg])
      return leg;

  return NO_SWING_LEG;
}

inline void HyqState::SetSwingleg(LegID leg)
{
  swingleg_ = false;
  swingleg_[leg] = true;
}

inline std::ostream& operator<<(std::ostream& out, const HyqState& hyq)
{
  out << "base: " << hyq.base_ << "\n"
      << "feet: " << "\tLF = " <<  hyq.feet_[LF] << "\n"
                  << "\tRF = " <<  hyq.feet_[RF] << "\n"
                  << "\tLH = " <<  hyq.feet_[LH] << "\n"
                  << "\tRH = " <<  hyq.feet_[RH] << "\n"
      << "swing:\t" << hyq.swingleg_ << "\n";
   return out;
}

