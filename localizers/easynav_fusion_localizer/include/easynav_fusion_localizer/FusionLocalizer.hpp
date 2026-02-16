#pragma once

#include <memory>

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_fusion_localizer/ukf_wrapper.hpp" // Tu wrapper refactorizado

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace easynav
{

/**
 * @class FusionLocalizer
 * @brief Plugin de localización para EasyNav que integra el UKF de
 * robot_localization para la fusión de sensores.
 */
class FusionLocalizer : public easynav::LocalizerMethodBase
{
public:
  FusionLocalizer() = default;
  virtual ~FusionLocalizer() = default;

protected:
  /**
   * @brief Hook de inicialización de MethodBase.
   *
   * Loads and initializes the UkfWrapper, which will load parameters
   * and create all robot_localization subscribers/publishers.
   *
   * @throws std::runtime_error if initialization fails.
   */
  void on_initialize() override;

  /**
   * @brief Hook de actualización RT (alta frecuencia) de LocalizerMethodBase.
   *
   * Esta función actúa como nuestro "timer" manual. Llama al ciclo
   * principal (periodicUpdate) del filtro UKF y actualiza el NavState
   * de easynav con la pose filtrada.
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief Hook de actualización no-RT (baja frecuencia).
   *
   * En esta implementación, toda la lógica principal reside en update_rt.
   */
  void update(NavState & nav_state) override;

private:
  std::unique_ptr<robot_localization::UkfWrapper> ukf_wrapper_;

  int n_imu_sensors_{0};
  int n_gps_sensors_{0};

  geometry_msgs::msg::PoseWithCovarianceStamped
  navsatfix_to_pose(const sensor_msgs::msg::NavSatFix & navsat_msg);

  double latitude_origin_{0.0};
  double longitude_origin_{0.0};
  double altitude_origin_{0.0};
  double UTM_origin_x_{0.0};
  double UTM_origin_y_{0.0};
  double UTM_origin_z_{0.0};
  std::string UTM_zone_;

  std::vector<double> last_gps_stamp_;

};

}  // namespace easynav
