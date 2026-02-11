#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq =
      Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}

#if CERES_VERSION_MAJOR >= 2
bool PoseLocalParameterization::PlusJacobian(const double *x,
                                             double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalParameterization::Minus(const double *x, const double *y,
                                      double *y_minus_x) const {
  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

  Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);
  delta.head<3>() = p_y - p_x;

  Eigen::Quaterniond q_diff = q_x.inverse() * q_y;
  // Convert quaternion difference to angle-axis (3-vector)
  Eigen::AngleAxisd aa(q_diff);
  delta.tail<3>() = aa.angle() * aa.axis();

  return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x,
                                              double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
  j.setZero();
  j.topLeftCorner<3, 3>().setIdentity();
  j.bottomRightCorner<3, 3>().setIdentity();
  // Simplified Jacobian - this is an approximation valid near identity
  return true;
}
#else
bool PoseLocalParameterization::ComputeJacobian(const double *x,
                                                double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}
#endif
