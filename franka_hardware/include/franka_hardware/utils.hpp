#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

template <typename Derived1, typename Derived2, typename Derived3>
void compute_twist(const Eigen::MatrixBase<Derived1>& first_cartesian_pose,
                   const Eigen::MatrixBase<Derived2>& second_cartesian_pose,
                   Eigen::MatrixBase<Derived3>& cartesian_twist,
                   const double time_delta) {
  static_assert(Derived1::RowsAtCompileTime == 4 && Derived1::ColsAtCompileTime == 4,
                "first_cartesian_pose must be a 4x4 matrix.");
  static_assert(Derived2::RowsAtCompileTime == 4 && Derived2::ColsAtCompileTime == 4,
                "second_cartesian_pose must be a 4x4 matrix.");
  static_assert(Derived3::RowsAtCompileTime == 6 && Derived3::ColsAtCompileTime == 1,
                "cartesian_twist must be a 6x1 vector.");

  // Extract linear and angular components from the poses
  const auto R = second_cartesian_pose.template block<3, 3>(0, 0);
  const auto R_last = first_cartesian_pose.template block<3, 3>(0, 0);
  const Eigen::Matrix3d& R_delta = R * R_last.transpose();

  // Convert to angle-axis (Eigen::AngleAxisd does not allocate memory for axis/angle)
  const Eigen::AngleAxisd angle_axis(R_delta);
  
  // Compute linear velocity (no allocation, head() returns a reference)
  cartesian_twist.template head<3>() =
      (second_cartesian_pose.template block<3, 1>(0, 3) - first_cartesian_pose.template block<3, 1>(0, 3)) /
      time_delta;

  // Compute angular velocity (no allocation, axis() returns a reference)
  cartesian_twist.template tail<3>() = angle_axis.axis() * angle_axis.angle() / time_delta;
}

template <typename Derived1, typename Derived2>
void calculate_delta_pose(const Eigen::MatrixBase<Derived1>& cartesian_twist,
             const double time_delta,
             Eigen::MatrixBase<Derived2>& delta_pose) {
  static_assert(Derived1::RowsAtCompileTime == 6 && Derived1::ColsAtCompileTime == 1,
        "cartesian_twist must be a 6x1 vector.");
  static_assert(Derived2::RowsAtCompileTime == 4 && Derived2::ColsAtCompileTime == 4,
        "delta_pose must be a 4x4 matrix.");

  const double norm = cartesian_twist.template tail<3>().norm();
  const double angle = norm * time_delta;
  Eigen::AngleAxisd angle_axis;
  if (norm > 1e-8) {
  angle_axis = Eigen::AngleAxisd(angle, cartesian_twist.template tail<3>().normalized());
  } else {
  angle_axis = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
  }

  delta_pose.setIdentity();
  delta_pose.template block<3,3>(0,0) = angle_axis.toRotationMatrix();
  delta_pose.template block<3,1>(0,3) = cartesian_twist.template head<3>() * time_delta;
}
