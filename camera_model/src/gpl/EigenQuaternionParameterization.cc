#include "camodocal/gpl/EigenQuaternionParameterization.h"

#include <cmath>

namespace camodocal {

bool EigenQuaternionParameterization::Plus(const double *x, const double *delta,
                                           double *x_plus_delta) const {
  const double norm_delta =
      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm_delta > 0.0) {
    const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
    double q_delta[4];
    q_delta[0] = sin_delta_by_delta * delta[0];
    q_delta[1] = sin_delta_by_delta * delta[1];
    q_delta[2] = sin_delta_by_delta * delta[2];
    q_delta[3] = cos(norm_delta);
    EigenQuaternionProduct(q_delta, x, x_plus_delta);
  } else {
    for (int i = 0; i < 4; ++i) {
      x_plus_delta[i] = x[i];
    }
  }
  return true;
}

#if CERES_VERSION_MAJOR >= 2
bool EigenQuaternionParameterization::PlusJacobian(const double *x,
                                                   double *jacobian) const {
  jacobian[0] = x[3];
  jacobian[1] = x[2];
  jacobian[2] = -x[1]; // NOLINT
  jacobian[3] = -x[2];
  jacobian[4] = x[3];
  jacobian[5] = x[0]; // NOLINT
  jacobian[6] = x[1];
  jacobian[7] = -x[0];
  jacobian[8] = x[3]; // NOLINT
  jacobian[9] = -x[0];
  jacobian[10] = -x[1];
  jacobian[11] = -x[2]; // NOLINT
  return true;
}

bool EigenQuaternionParameterization::Minus(const double *x, const double *y,
                                            double *y_minus_x) const {
  // Compute x^{-1} * y
  double x_inv[4] = {-x[0], -x[1], -x[2], x[3]};
  double diff[4];
  EigenQuaternionProduct(x_inv, y, diff);
  // Convert to angle-axis
  double sin_sq = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
  double sin_theta = sqrt(sin_sq);
  if (sin_theta > 0.0) {
    double cos_theta = diff[3];
    double two_theta = 2.0 * ((cos_theta < 0.0) ? atan2(-sin_theta, -cos_theta)
                                                : atan2(sin_theta, cos_theta));
    double k = two_theta / sin_theta;
    y_minus_x[0] = diff[0] * k;
    y_minus_x[1] = diff[1] * k;
    y_minus_x[2] = diff[2] * k;
  } else {
    y_minus_x[0] = 2.0 * diff[0];
    y_minus_x[1] = 2.0 * diff[1];
    y_minus_x[2] = 2.0 * diff[2];
  }
  return true;
}

bool EigenQuaternionParameterization::MinusJacobian(const double *x,
                                                    double *jacobian) const {
  // Simplified Jacobian (identity-like in tangent space)
  // J is 3x4 (TangentSize x AmbientSize), row-major
  jacobian[0] = x[3];
  jacobian[1] = -x[2];
  jacobian[2] = x[1];
  jacobian[3] = -x[0];
  jacobian[4] = x[2];
  jacobian[5] = x[3];
  jacobian[6] = -x[0];
  jacobian[7] = -x[1];
  jacobian[8] = -x[1];
  jacobian[9] = x[0];
  jacobian[10] = x[3];
  jacobian[11] = -x[2];
  return true;
}
#else
bool EigenQuaternionParameterization::ComputeJacobian(const double *x,
                                                      double *jacobian) const {
  jacobian[0] = x[3];
  jacobian[1] = x[2];
  jacobian[2] = -x[1]; // NOLINT
  jacobian[3] = -x[2];
  jacobian[4] = x[3];
  jacobian[5] = x[0]; // NOLINT
  jacobian[6] = x[1];
  jacobian[7] = -x[0];
  jacobian[8] = x[3]; // NOLINT
  jacobian[9] = -x[0];
  jacobian[10] = -x[1];
  jacobian[11] = -x[2]; // NOLINT
  return true;
}
#endif

} // namespace camodocal
