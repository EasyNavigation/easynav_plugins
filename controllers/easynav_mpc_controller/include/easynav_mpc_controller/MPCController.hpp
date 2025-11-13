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
#include <cmath>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

struct MPCParameters{
  Eigen::Vector2d goal;
  Eigen::Vector3d x0;
  Eigen::Matrix2d Q;
  Eigen::Matrix2d R;
  Eigen::Matrix2d Rd;
  double qtheta;
  int N;
  double dt;
};

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

protected:
  int horizon_steps_{5};      ///< Prediction horizon for MPC.
  double dt_{0.1};            ///< Time step for MPC.
  double max_lin_vel_{1.5};   ///< Maximum linear velocity for MPC.
  double max_ang_vel_{1.5};   ///< Maximum angular velocity for MPC.

private:
  Eigen::Matrix2d Q_ {{2.0, 0.0},{0.0, 2.0}};
  Eigen::Matrix2d R_ {{0.05, 0.0},{0.0, 0.05}};
  Eigen::Matrix2d Rd_ {{0.1, 0.0},{0.0, 0.1}};
  double qtheta_ {0.5};
  geometry_msgs::msg::TwistStamped cmd_vel_;  ///< Current velocity command.

};

}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__MPCCONTROLLER_HPP_
