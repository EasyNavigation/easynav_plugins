// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.

#ifndef EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_
#define EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_

#include <memory>
#include <expected>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <nlopt.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// \brief A MPC Controller.
class MPCController : public ControllerMethodBase
{
public:
  MPCController();

  /// \brief Destructor.
  ~MPCController() override;

  /// \brief Initializes parameters and MPC controller.
  /// \return std::expected<void, std::string> success or error message.
  std::expected<void, std::string> on_initialize() override;

  /// \brief Updates the controller using the given NavState.
  /// \param nav_state Current navigation state, including odometry and planned path.
  void update_rt(NavState & nav_state) override;

  /// \brief Predict the kinematic model for the robot.
  /// \param x Current state, x, y and theta angle.
  /// \param v Linear velocity command.
  /// \param w Angular velocity command.
  Eigen::Vector3d kinematic_model(const Eigen::Vector3d &x, double v, double w);

  /// \brief Cost function to optimize
  /// \param u
  /// \param grad
  /// \param data
  double cost_function(const std::vector<double> &u, std::vector<double> &grad, void *data);

  struct MPCParameters{
    Eigen::Vector2d goal;
    Eigen::Vector3d x0;
  };

  struct NLoptCallbackData {
    MPCController *controller;
    void *user_data;
  };

protected:
  int horizon_steps_{5};      ///< Prediction horizon for MPC.
  double dt_{0.1};            ///< Time step for MPC.
  double max_lin_vel_{1.5};   ///< Maximum linear velocity for MPC.
  double max_ang_vel_{1.5};   ///< Maximum angular velocity for MPC.

private:
  geometry_msgs::msg::TwistStamped cmd_vel_;  ///< Current velocity command.

};

}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_
