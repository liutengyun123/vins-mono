#pragma once

#include "../utility/utility.h"
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

#if CERES_VERSION_MAJOR >= 2
class PoseLocalParameterization : public ceres::Manifold {
public:
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const override;
  virtual bool PlusJacobian(const double *x, double *jacobian) const override;
  virtual bool Minus(const double *x, const double *y,
                     double *y_minus_x) const override;
  virtual bool MinusJacobian(const double *x, double *jacobian) const override;
  virtual int AmbientSize() const override { return 7; }
  virtual int TangentSize() const override { return 6; }
};
#else
class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};
#endif
