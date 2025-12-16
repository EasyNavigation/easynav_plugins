#include "easynav_fusion_localizer/FusionLocalizer.hpp"

#include "easynav_localizer/LocalizerNode.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_common/types/IMUPerception.hpp"
#include "easynav_common/types/GNSSPerception.hpp"

#include <GeographicLib/UTMUPS.hpp>

#include "easynav_common/YTSession.hpp"

namespace easynav
{

void FusionLocalizer::on_initialize()
{

  try {

    auto node = get_node();

    auto localizer_node = std::dynamic_pointer_cast<LocalizerNode>(node);

    last_gps_stamp_.resize(10, 0.0);

    const std::string & plugin_name = this->get_plugin_name();
    const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

    RCLCPP_INFO(localizer_node->get_logger(), "Using tf_prefix: '%s'", tf_info.tf_prefix.c_str());
    RCLCPP_INFO(localizer_node->get_logger(), "Using parameter namespace: '%s'",
    plugin_name.c_str());

    ukf_wrapper_ = std::make_unique<robot_localization::UkfWrapper>(
      localizer_node, tf_info.tf_prefix, plugin_name + ".local_filter"
    );
    ukf_wrapper_->initialize();
    localizer_node->declare_parameter(plugin_name + ".latitude_origin", double(0.0));
    localizer_node->get_parameter(plugin_name + ".latitude_origin", latitude_origin_);

    localizer_node->declare_parameter(plugin_name + ".longitude_origin", double(0.0));
    localizer_node->get_parameter(plugin_name + ".longitude_origin", longitude_origin_);

    localizer_node->declare_parameter(plugin_name + ".altitude_origin", double(0.0));
    localizer_node->get_parameter(plugin_name + ".altitude_origin", altitude_origin_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "Critical failure initializing UkfWrapper: %s",
      e.what());
    // Raise error
    throw std::runtime_error(std::string("Failed to initialize UkfWrapper: ") + e.what());
  }


  int zone;
  bool northp;

  GeographicLib::UTMUPS::Forward(latitude_origin_, longitude_origin_, zone, northp, UTM_origin_x_,
      UTM_origin_y_);
  UTM_zone_ = std::to_string(zone) + (northp ? "N" : "S");
  UTM_origin_z_ = altitude_origin_;

  n_gps_sensors_ = static_cast<int>(ukf_wrapper_->getGpsCallbackDataArr().size());

  RCLCPP_INFO(get_node()->get_logger(), "FusionLocalizer (UKF) initialized successfully.");
}

// 2. Hook de actualización RT (Tu "Timer" de alta frecuencia)
void FusionLocalizer::update_rt(NavState & nav_state)
{
  if(n_gps_sensors_ && nav_state.has("gnss")) {
    auto gps_data = nav_state.get<GNSSPerceptions>(std::string("gnss"));
    for (int i = 0; i < n_gps_sensors_; ++i) {
      double gps_time = gps_data[i]->data.header.stamp.sec +
        gps_data[i]->data.header.stamp.nanosec * 1e-9;
      if (gps_time > last_gps_stamp_[i]) {
      // if(true) {
        last_gps_stamp_[i] = gps_time;
        auto pose = navsatfix_to_pose(gps_data[i]->data);
        // nav_state.set("UTM_gnss_pose", pose);
        // Call the wrapper callback
        const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
        ukf_wrapper_->poseCallback(
          std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(pose),
          ukf_wrapper_->getGpsCallbackDataArr()[i], // callback_data
          tf_info.map_frame, // target_frame
          tf_info.odom_frame,  // pose_source_frame
          false                           // imu_data
        );
      }
    }
  }

  ukf_wrapper_->periodicUpdate();

  nav_msgs::msg::Odometry current_odom;
  if (ukf_wrapper_->getFilteredOdometryMessage(&current_odom)) {
    nav_state.set("robot_pose", current_odom);
  }
}

// 3. Hook de actualización no-RT (baja frecuencia)
void FusionLocalizer::update([[maybe_unused]] NavState & nav_state)
{

}

geometry_msgs::msg::PoseWithCovarianceStamped FusionLocalizer::navsatfix_to_pose(
  const sensor_msgs::msg::NavSatFix & navsat_msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  // 1. Establecer el Header
  // Usamos el mismo timestamp que el mensaje original
  // y el world_frame_id que el filtro UKF espera (p.ej., "map" u "odom")
  pose_msg.header = navsat_msg.header;

  pose_msg.header.frame_id = tf_info.map_frame;

  // 2. Convertir coordenadas (Lat, Lon) a UTM (x, y)
  double utm_x, utm_y;
  int zone;
  bool northp;

  GeographicLib::UTMUPS::Forward(
    navsat_msg.latitude,
    navsat_msg.longitude,
    zone,
    northp,
    utm_x,
    utm_y);

  pose_msg.pose.pose.position.x = utm_x - UTM_origin_x_;
  pose_msg.pose.pose.position.y = utm_y - UTM_origin_y_;
  pose_msg.pose.pose.position.z = navsat_msg.altitude - UTM_origin_z_;

  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = 0.0;
  pose_msg.pose.pose.orientation.w = 1.0;

  pose_msg.pose.covariance.fill(0.0);

  pose_msg.pose.covariance[0] = navsat_msg.position_covariance[0];  // xx
  pose_msg.pose.covariance[1] = navsat_msg.position_covariance[1];  // xy
  pose_msg.pose.covariance[2] = navsat_msg.position_covariance[2];  // xz

  pose_msg.pose.covariance[6] = navsat_msg.position_covariance[3];  // yx
  pose_msg.pose.covariance[7] = navsat_msg.position_covariance[4];  // yy
  pose_msg.pose.covariance[8] = navsat_msg.position_covariance[5];  // yz

  pose_msg.pose.covariance[12] = navsat_msg.position_covariance[6]; // zx
  pose_msg.pose.covariance[13] = navsat_msg.position_covariance[7]; // zy
  pose_msg.pose.covariance[14] = navsat_msg.position_covariance[8]; // zz

  pose_msg.pose.covariance[21] = 99999.0;
  pose_msg.pose.covariance[28] = 99999.0;
  pose_msg.pose.covariance[35] = 99999.0;

  return pose_msg;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::FusionLocalizer, easynav::LocalizerMethodBase)
