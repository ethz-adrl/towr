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

  HeightMap ();
  virtual ~HeightMap ();

  virtual double GetHeight(double x, double y) const = 0;
  virtual double GetHeightDerivWrtX(double x, double y) const = 0;
  virtual double GetHeightDerivWrtY(double x, double y) const = 0;

  enum ID { Flat=0, Stairs, Gap };
  static Ptr MakeTerrain(ID type);
};


class HeightMapFlat : public HeightMap {
public:
  HeightMapFlat() {};
  ~HeightMapFlat() {};

  virtual double GetHeight(double x, double y)          const override { return height_; };
  virtual double GetHeightDerivWrtX(double x, double y) const override { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const override { return 0.0; };

private:
  double height_ = 0.0; // [m]
};


class HeightMapStairs : public HeightMap {
public:
  HeightMapStairs();
  ~HeightMapStairs();

  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;


private:
  double slope_;
  double slope_start_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_ */
