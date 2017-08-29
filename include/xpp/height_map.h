/**
 @file    height_map.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_
#define XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_

#include <memory>

namespace xpp {
namespace opt {


class HeightMap {
public:
  using Ptr = std::shared_ptr<HeightMap>;

  virtual ~HeightMap () {};

  virtual double GetHeight(double x, double y) const = 0;
  virtual double GetHeightDerivWrtX(double x, double y) const = 0;
  virtual double GetHeightDerivWrtY(double x, double y) const = 0;

  enum ID { FlatID=0, StairsID, GapID, SlopeID, K_TERRAIN_COUNT };
  static Ptr MakeTerrain(ID type);
};


class FlatGround : public HeightMap {
public:
  virtual double GetHeight(double x, double y)          const override { return height_; };
  virtual double GetHeightDerivWrtX(double x, double y) const override { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const override { return 0.0; };

private:
  double height_ = 0.0; // [m]
};


class Stairs : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;


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
  virtual double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double gap_start_ = 1.0;
  const double gap_width_ = 0.5;
  const double gap_depth_ = 3.0;
};


class Slope : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double slope_start_ = 0.5;
  double slope_ = 0.3;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_ */
