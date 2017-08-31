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
  using Ptr         = std::shared_ptr<HeightMap>;
  using Vector3d    = Eigen::Vector3d;
  using Derivatives = std::vector<Coords2D>;

  enum ID { FlatID=0, StairsID, GapID, SlopeID, ChimneyID, K_TERRAIN_COUNT };
  static Ptr MakeTerrain(ID type);
  virtual ~HeightMap () {};

  enum BasisVector { Normal, Tangent1, Tangent2 };

  virtual double GetHeight(double x, double y) const = 0;
  double GetDerivativeOfHeightWrt(Coords2D dim, double x, double y) const;
  Vector3d GetNormalizedBasis(BasisVector, double x, double y) const;
  Vector3d GetDerivativeOfNormalizedBasisWrt(BasisVector, Coords2D dim, double x, double y) const;

private:
  // not normalized basis vectors of basis vector derivatives
  Vector3d GetBasisNotNormalized(BasisVector, double x, double y, const Derivatives& = {}) const;
  Vector3d GetNormalNotNormalized(double x, double y, const Derivatives& = {}) const;
  Vector3d GetTangent1NotNormalized(double x, double y, const Derivatives& = {}) const;
  Vector3d GetTangent2NotNormalized(double x, double y, const Derivatives& = {}) const;


  // first derivaties that must be implemented by the user
  virtual double GetHeightDerivWrtX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const { return 0.0; };

  // second derivatives wrt first letter, then second
  double GetSecondDerivativeOfHeightWrt(Coords2D dim1, Coords2D dim2, double x, double y) const;
  virtual double GetHeightDerivWrtXX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtXY(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYY(double x, double y) const { return 0.0; };


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
