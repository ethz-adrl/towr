/**
 @file    height_map.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_
#define XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_

#include <Eigen/Dense>
#include <memory>

#include "cartesian_declarations.h"

namespace xpp {
namespace opt {


class HeightMap {
public:
  using Ptr      = std::shared_ptr<HeightMap>;
  using Vector3d = Eigen::Vector3d;

  enum ID { FlatID=0, StairsID, GapID, SlopeID, ChimneyID, K_TERRAIN_COUNT };
  static Ptr MakeTerrain(ID type);
  virtual ~HeightMap () {};

  virtual double GetHeight(double x, double y) const = 0;
  double GetHeightFirstDerivativeWrt(Coords3D dim, double x, double y) const;



  // Attention: these are not normalized!
  Vector3d GetNormal(double x, double y, bool normalize=true) const;
  Vector3d GetNormalDerivativeWrt(Coords3D dim, double x, double y) const;
//  Vector3d GetNormalDerivativeWrtX(double x, double y) const;
//  Vector3d GetNormalDerivativeWrtY(double x, double y) const;

  Vector3d GetTangent1(double x, double y) const;
  Vector3d GetTangent1DerivativeWrtX(double x, double y) const;
  Vector3d GetTangent1DerivativeWrtY(double x, double y) const;

  Vector3d GetTangent2(double x, double y) const;
  Vector3d GetTangent2DerivativeWrtX(double x, double y) const;
  Vector3d GetTangent2DerivativeWrtY(double x, double y) const;


private:

  Vector3d GetNonNormalizedNormalDerivativeWrt(Coords3D dim, double x, double y) const;

  // spring_clean_ use Coords2D to better show intentions
  double GetHeightSecondDerivative(Coords3D dim1, Coords3D dim2, double x, double y) const;


  // first derivaties that must be implemented by the user
  virtual double GetHeightDerivWrtX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const { return 0.0; };

  // second derivatives wrt first letter, then second
  virtual double GetHeightDerivWrtXX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtXY(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYY(double x, double y) const { return 0.0; };


  double GetOuterDerivativeWithValInNumerator(double val) const;
  double GetOuterDerivativeWithOneInNumerator(double val) const;

  Vector3d GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(const Vector3d& non_normalized, int index) const;
};


class FlatGround : public HeightMap {
public:
  virtual double GetHeight(double x, double y)  const override { return height_; };

private:
  double height_ = 0.0; // [m]
};


class Stairs : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;

private:
  double first_step_start_  = 0.7;
  double first_step_width_  = 0.4;
  double height_first_step  = 0.2;
  double height_second_step = 0.4;
  double width_top = 1.0;
};


class Gap : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 1.0;
  const double gap_width_ = 0.5;
  const double gap_depth_ = 3.0;
};


class Slope : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 0.5;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.5;

  const double x_down_start_ = slope_start_+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center/up_length_;
};



class Chimney : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;
  virtual double GetHeightDerivWrtYY(double x, double y) const override;

private:
  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.30; // distance to start of slope from center at z=0
  const double slope_   = 3;
  const double z_depth_ = 2.5;
};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_ */
