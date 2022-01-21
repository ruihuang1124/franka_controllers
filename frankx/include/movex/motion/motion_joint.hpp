#pragma once

#include <Eigen/Core>


namespace movex {

/**
* A motion in the joint space
*/
  struct JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d target;

    explicit JointMotion(const std::array<double, 7> target): target(target.data()) { }
  };

/**
* A motion described in velocity in the joint space
*/
  struct JointVelocityMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d target_position_;
    const Vector7d target_velocity_;

    explicit JointVelocityMotion(const std::array<double, 7> target_position, const std::array<double, 7> target_velocity)
        : target_position_(target_position.data()), target_velocity_(target_velocity.data()) { }
  };

} // namespace movex
