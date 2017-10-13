/**
 @file    motion_parameter_instances.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Specific values of the motion parameters corresponding to robots.
 */

#include <xpp/motion_parameter_instances.h>

#include <initializer_list>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <xpp/endeffectors.h>

namespace xpp {
namespace opt {


MonopedOptParameters::MonopedOptParameters()
{
  order_coeff_polys_ = 4;
  dt_base_polynomial_ = 0.1;

  force_splines_per_stance_phase_ = 3;

  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 1;

  double f  = 0.2;
  double c = 0.2;
  contact_timings_ = ContactTimings(1);
  contact_timings_.at(E0) = {c, f, c, f, c, f, c, f, c, c, f, c, f, c, f, c, f, c};

  min_phase_duration_ = 0.1;
  max_phase_duration_ = GetTotalTime();
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();


  constraints_ = {
      BasePoly,
      RomBox,
      Dynamic,
      Terrain,
      TotalTime,
  };

}

BipedOptParameters::BipedOptParameters()
{
  order_coeff_polys_ = 4;
  dt_base_polynomial_ = 0.1;

  ee_splines_per_swing_phase_ = 1;
  force_splines_per_stance_phase_ = 3;


  dt_range_of_motion_ = 0.1;


  double f  = 0.2;
  double c = 0.2;
  double offset = c;
  contact_timings_ = ContactTimings(2);
  contact_timings_.at(E0) = {c+offset,f,c,f,c,f,c,f,c,f,c,f,c,f, c};
  contact_timings_.at(E1) = {       c,f,c,f,c,f,c,f,c,f,c,f,c,f, c+offset};


  min_phase_duration_ = 0.1;
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();
  max_phase_duration_ = GetTotalTime();


  constraints_ = {
      BasePoly,
      RomBox,
      Dynamic,
      Terrain,
      TotalTime,
  };
}



QuadrupedOptParameters::QuadrupedOptParameters ()
{
  using namespace xpp::quad;

  order_coeff_polys_  = 4; // used only with coeff_spline representation
  dt_base_polynomial_ = 0.15;


  force_splines_per_stance_phase_ = 3;


  // range of motion constraint
  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 1;



  double f = 0.2; // [s] t_free
  double c = 0.2; // [s] t_contact
  double t_offset = f;
  contact_timings_ = ContactTimings(4);
  auto m = Reverse(kMapOptToQuad);
  contact_timings_.at(m.at(LH)) = {t_offset + c, f, c, f, c, f, c, f, c, f, c, f, c           };
  contact_timings_.at(m.at(RH)) = {           c, f, c, f, c, f, c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(LF)) = {           c, f, c, f, c, f, c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(RF)) = {t_offset + c, f, c, f, c, f, c, f, c, f, c, f, c           };


  min_phase_duration_ = 0.1;
  max_phase_duration_ = GetTotalTime();
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();

  constraints_ = {
      BasePoly,
      RomBox,
      Dynamic,
      Terrain,
      TotalTime,
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}


QuadrotorOptParameters::QuadrotorOptParameters ()
{
  using namespace xpp::quad;

  order_coeff_polys_  = 4;
  dt_base_polynomial_ = 0.2; //s 0.05


  force_splines_per_stance_phase_ = 3;


  // range of motion constraint
  dt_range_of_motion_ = 0.1;
  ee_splines_per_swing_phase_ = 1;



  double f = 0.2; // [s] t_free
  double c = 0.2; // [s] t_contact
  double t_offset = f;
  contact_timings_ = ContactTimings(4);
  auto m = Reverse(kMapOptToRotor);
  contact_timings_.at(m.at(L)) = {t_offset + c, f, c, f, c, f, c           };
  contact_timings_.at(m.at(R)) = {           c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(F)) = {           c, f, c, f, c, f, c + t_offset};
  contact_timings_.at(m.at(H)) = {t_offset + c, f, c, f, c, f, c           };

  min_phase_duration_ = 0.1;
  max_phase_duration_ = GetTotalTime();
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();

  constraints_ = {
      //BasePoly
      RomBox,
      Dynamic,
//      TotalTime,
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}



//  II_.SetCount(robot_ee_.size()); II_.SetAll(false);
//  PI_.SetCount(robot_ee_.size()); PI_.SetAll(false);
//  bI_.SetCount(robot_ee_.size()); bI_.SetAll(false);
//  IP_.SetCount(robot_ee_.size()); IP_.SetAll(false);
//  Ib_.SetCount(robot_ee_.size()); Ib_.SetAll(false);
//  // 2 swinglegs
//  Pb_.SetCount(robot_ee_.size()); Pb_.SetAll(false);
//  bP_.SetCount(robot_ee_.size()); bP_.SetAll(false);
//  BI_.SetCount(robot_ee_.size()); BI_.SetAll(false);
//  IB_.SetCount(robot_ee_.size()); IB_.SetAll(false);
//  PP_.SetCount(robot_ee_.size()); PP_.SetAll(false);
//  bb_.SetCount(robot_ee_.size()); bb_.SetAll(false);
//  // 3 swinglegs
//  Bb_.SetCount(robot_ee_.size()); Bb_.SetAll(false);
//  BP_.SetCount(robot_ee_.size()); BP_.SetAll(false);
//  bB_.SetCount(robot_ee_.size()); bB_.SetAll(false);
//  PB_.SetCount(robot_ee_.size()); PB_.SetAll(false);
//  // flight-phase
//  BB_.SetCount(robot_ee_.size()); BB_.SetAll(true);
//
//  PI_.At(kMapQuadToOpt.at(LH)) = true;
//  bI_.At(kMapQuadToOpt.at(RH)) = true;
//  IP_.At(kMapQuadToOpt.at(LF)) = true;
//  Ib_.At(kMapQuadToOpt.at(RF)) = true;
//  // two swinglegs
//  Pb_.At(kMapQuadToOpt.at(LH)) = true; Pb_.At(kMapQuadToOpt.at(RF)) = true;
//  bP_.At(kMapQuadToOpt.at(RH)) = true; bP_.At(kMapQuadToOpt.at(LF)) = true;
//  BI_.At(kMapQuadToOpt.at(LH)) = true; BI_.At(kMapQuadToOpt.at(RH)) = true;
//  IB_.At(kMapQuadToOpt.at(LF)) = true; IB_.At(kMapQuadToOpt.at(RF)) = true;
//  PP_.At(kMapQuadToOpt.at(LH)) = true; PP_.At(kMapQuadToOpt.at(LF)) = true;
//  bb_.At(kMapQuadToOpt.at(RH)) = true; bb_.At(kMapQuadToOpt.at(RF)) = true;
//  // three swinglegs
//  Bb_.At(kMapQuadToOpt.at(LH)) = true; Bb_.At(kMapQuadToOpt.at(RH)) = true;  Bb_.At(kMapQuadToOpt.at(RF))= true;
//  BP_.At(kMapQuadToOpt.at(LH)) = true; BP_.At(kMapQuadToOpt.at(RH)) = true;  BP_.At(kMapQuadToOpt.at(LF))= true;
//  bB_.At(kMapQuadToOpt.at(RH)) = true; bB_.At(kMapQuadToOpt.at(LF)) = true;  bB_.At(kMapQuadToOpt.at(RF))= true;
//  PB_.At(kMapQuadToOpt.at(LH)) = true; PB_.At(kMapQuadToOpt.at(LF)) = true;  PB_.At(kMapQuadToOpt.at(RF))= true;


//QuadrupedMotionParameters::MotionTypePtr
//QuadrupedMotionParameters::MakeMotion (opt::MotionTypeID id)
//{
//  switch (id) {
//    case WalkID:
//      return std::make_shared<Walk>();
//    case TrotID:
//      return std::make_shared<Trot>();
//    case BoundID:
//      return std::make_shared<Bound>();
//    case PaceID:
//      return std::make_shared<Pace>();
//    case PushRecID:
//      return std::make_shared<PushRecovery>();
//    default:
//      throw std::runtime_error("MotionTypeID not defined");
//  }
//}

//Walk::Walk()
//{
//  max_dev_xy_ << 0.15, 0.15, 0.1;
////  id_ = opt::WalkID;
//
////  double t_phase = 0.3;
////  double t_trans = 0.1;
////  contact_timings_ =
////  {
////      0.3,
////      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
////      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
//////      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
////      0.2,
////  };
////  contact_sequence_ =
////  {
////      II_,
////      PI_, PP_, IP_, bI_, bb_, Ib_,
////      PI_, PP_, IP_, bI_, bb_, Ib_,
//////      PI_, PP_, IP_, bI_, bb_, Ib_,
////      II_,
////  };
//
//
//  double t_step = 0.4;
//  contact_timings_ =
//  {
//      0.4,
//      t_step, t_step,t_step,t_step,
////      t_step, t_step,t_step,t_step,
////      t_step, t_step,t_step,t_step,
//      0.2,
//  };
//
//  contact_sequence_ =
//  {
//      II_,
//      PI_, IP_, bI_, Ib_,
////      PI_, IP_, bI_, Ib_,
////      PI_, IP_, bI_, Ib_,
//      II_,
//  };
//
//
//  constraints_ = { State,
//                   JunctionCom,
//                   Dynamic,
//                   RomBox, // usually enforced as soft-constraint/cost
//  };
////
////
////  cost_weights_[ComCostID]          = 1.0;
////  cost_weights_[RangOfMotionCostID] = 1.0;
////  cost_weights_[PolyCenterCostID]   = 1.0;
////  cost_weights_[FinalComCostID] = 1000.0;
//}
//
//Trot::Trot()
//{
//  max_dev_xy_ << 0.2, 0.2, 0.1;
////  id_ = opt::TrotID;
//
//  double t_phase = 0.3;
//  double t_trans = 0.1;
//
//  contact_timings_ =
//  {   0.3,
//      t_phase, t_phase,
////      t_phase, t_phase, // trot
////      0.3, // flight_phase
//////      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase, // walk
////      t_phase, t_phase, t_phase, t_phase, // trot
//      0.3
//  };
//
//  contact_sequence_ =
//  {
//      II_,
//      bP_, Pb_,
////      bP_, Pb_, // trot
////      BB_, // flight-phase
//////      PI_, PP_, IP_, bI_, bb_, Ib_, // walk
////      bP_, Pb_, bP_, Pb_, // trot
//      II_
//  };
//
//
//  constraints_ = {
//                   RomBox, // usually enforced as soft-constraint/cost
////                   State,
////                   JunctionCom,
////                   Dynamic
//  };
////
////  cost_weights_[RangOfMotionCostID] = 10.0;
////  cost_weights_[ComCostID]      = 1.0;
//
////  cost_weights_[FinalComCostID] = 1.0;
////  cost_weights_[PolyCenterCostID]   = 50.0;
//}
//
//Pace::Pace()
//{
//  max_dev_xy_ << 0.2, 0.2, 0.1;
////  id_ = opt::PaceID;
//
//  double t_flight = 0.2;
//
//  contact_timings_ =
//  {
//      0.3,
//      0.3,
//      t_flight, // jump
//      0.3,
//      t_flight, // jump
//      0.3,
//      t_flight, // jump
//      0.3,
//      0.3
//  };
//  contact_sequence_ =
//  {
//      II_,
//      PP_,
//      BB_, // jump
//      bb_,
//      BB_, // jump
//      PP_,
//      BB_, // jump
//      bb_,
//      II_,
//  };
//
//  constraints_ = { State,
//                   JunctionCom,
//                   Dynamic,
//                   RomBox, // usually enforced as soft-constraint/cost
//  };
//
////  cost_weights_[RangOfMotionCostID] = 10.0;
////  cost_weights_[ComCostID]          = 1.0;
////  cost_weights_[PolyCenterCostID]   = 0.0;
//}
//
//Bound::Bound()
//{
//  max_dev_xy_ << 0.25, 0.21, 0.18;
////  id_ = opt::BoundID;
//
//
////  // sequence for normal bound
////  contact_sequence_ =
////  {
////      II_,
////      BI_,
////      IB_,
////      BB_, // jump
////      BI_,
////      IB_,
////      BB_, // jump
////      BI_,
////      IB_,
////      II_
////  };
////
////  contact_timings_ =
////  {
////      0.8,
////      0.4,
////      0.3,
////      0.2, // jump
////      0.4,
////      0.3,
////      0.2, // jump
////      0.4,
////      0.3,
////      0.3
////  };
//
//  // sequence for 4 feet jumps
//  contact_sequence_ =
//  {
//      II_,
//      BB_,
//      II_,
//      BB_,
//      II_,
//      BB_,
//      II_
//  };
//
//  contact_timings_ =
//  {
//      0.3,
//      0.3,
//      0.3,
//      0.6,
//      0.3,
//      0.3,
//      0.3
//  };
//
//
//
////    // sequence 3-1
////  contact_sequence_ =
////  {
////      II_,
////      BP_, Ib_, BP_, Ib_,
////      BP_, Ib_, BP_, Ib_,
////      BP_, Ib_, BP_, Ib_,
////      II_
////  };
////  contact_timings_ =
////  {
////      0.3,
////      0.3,0.3,0.3,0.3,
////      0.3,0.3,0.3,0.3,
////      0.3,0.3,0.3,0.3,
////      0.3
////  };
//
//
////  // sequence for limping, keeping left hind up all the time
////  contact_sequence_ =
////  {
////      II_,
////      Bb_, bP_, Bb_, bP_, // right hind leg stationary
////      Bb_, bP_, Bb_, bP_, // right hind leg stationary
////      Bb_, bP_, Bb_, bP_, // right hind leg stationary
////      II_
////  };
////
////
//
//
////  // biped walk
////  contact_sequence_ =
////  {
////      II_,
////      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
////      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
////      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
////      II_
////  };
////  contact_timings_ =
////  {
////      0.3,
////      0.3,0.3,0.3,0.3,
////      0.3,0.3,0.3,0.3,
////      0.3,0.3,0.3,0.3,
////      0.3
////  };
//
//
//
//
//  constraints_ = { State,
//                   JunctionCom,
//                   Dynamic,
//                   RomBox, // usually enforced as soft-constraint/cost
//  };
//
////  cost_weights_[RangOfMotionCostID] = 10.0;
////  cost_weights_[ComCostID]          = 1.0;
////  cost_weights_[PolyCenterCostID]   = 0.0;
//}
//
//PushRecovery::PushRecovery ()
//{
//  max_dev_xy_ << 0.2, 0.2, 0.1;
////  id_ = opt::PushRecID;
//
//  SetContactSequence(0.0, 0.0);
//
//  constraints_ = { State,
//                   JunctionCom,
//                   Dynamic,
//                   RomBox,
//  };
//
////  cost_weights_[ComCostID]          = 1.0;
////  cost_weights_[RangOfMotionCostID] = 100.0;
////  cost_weights_[FinalComCostID] = 1000.0;
////  cost_weights_[PolyCenterCostID]   = 0.0;
//}
//
//void
//QuadrupedMotionParameters::SetContactSequence (double p_overlap_lateral,
//                                               double p_overlap_opposite)
//{
//  // distribution of swing motion to different phases
//  double p_single_leg  = 1.0-p_overlap_lateral-p_overlap_opposite;
//
//  double t = 0.3;
//  double ts = t*p_single_leg;
//  double tl = t*p_overlap_lateral;
//  double to = t*p_overlap_opposite;
//
//
//  ContactSequence lateral  = {PI_, PP_, IP_, bP_, bI_, bb_, Ib_, Pb_};
//  ContactSequence circular = {PI_, PP_, IP_, IB_, Ib_, bb_, bI_, BI_};
//  ContactTimings  timings = { ts,  tl,  ts,  to,  ts,  tl,  ts,  to};
//
//
//  ContactSequence sequence = lateral;
//
//  contact_timings_.clear();
//  contact_sequence_.clear();
//
//  // beginning of motion
//  contact_timings_.push_back(0.3);
//  contact_sequence_.push_back(II_);
//
//  contact_timings_.push_back(to);
//  contact_sequence_.push_back(PI_);
//
//  // during the motion
//  double sign = +1;
//  for (int i=0; i<12; ++i) {
//
//    for (int j=0; j<sequence.size(); ++j) {
//
//      double duration = timings.at(j);
//      if (duration > 1e-10) { // skip phases with zero duration
//        contact_sequence_.push_back(sequence.at(j));
//        contact_timings_.push_back(duration);
//      }
//
//
//      //    contact_sequence_.insert(contact_sequence_.end(), sequence.begin(), sequence.end());
//      //    contact_timings_.insert(contact_timings_.end(), timings.begin(), timings.end());
//
//      // adapt timings
//
////      p_overlap_lateral+= sign*0.01;
//
////      double r1 = ((double) rand() / (RAND_MAX));
////      double r2 = ((double) rand() / (RAND_MAX));
//
////      p_overlap_opposite = r1/0.5 + 0.25;
////      p_overlap_lateral  = r2/(1-p_overlap_opposite);
//
//      int n_overlaps = 1;
//
//      p_overlap_opposite+= sign*0.025/n_overlaps;
////      p_overlap_lateral += sign*0.025/n_overlaps;
//
//      if (p_overlap_opposite > 1.3/n_overlaps) {
//        p_overlap_opposite = 1.3/n_overlaps;
////        p_overlap_lateral = 1.3/n_overlaps;
//        sign = -1;
//      }
//
//
//      p_single_leg  = 1.0-p_overlap_lateral-p_overlap_opposite;
//
//      ts = t*p_single_leg;
//      tl = t*p_overlap_lateral;
//      to = t*p_overlap_opposite;
//      timings = { ts,  tl,  ts,  to,  ts,  tl,  ts,  to};
//    }
//  }
//
//  // end of motion
//  // remove last transition
//  contact_timings_.pop_back();
//  contact_sequence_.pop_back();
//
//  contact_timings_.push_back(0.4);
//  contact_sequence_.push_back(sequence.end()[-2]);
//
//  contact_timings_.push_back(0.6);
//  contact_sequence_.push_back(II_);
//
////  contact_sequence_ =
////  {
////      II_,
////      PI_,
////      //
////      PI_, PP_, IP_, // left side
////      bP_,           // transition left-right
////      bI_, bb_, Ib_, // right side
////      Pb_,           // transition right-left
////      //
////      PI_, PP_, IP_, // left side
////      bP_,           // transition left-right
////      bI_, bb_, Ib_, // right side
////      Pb_,           // transition right-left
////      //
////      PI_, PP_, IP_, // left side
////      bP_,           // transition left-right
////      bI_, bb_, Ib_, // right side
////      //
////      Ib_,
////      II_
////  };
//}

} // namespace opt
}// namespace xpp
