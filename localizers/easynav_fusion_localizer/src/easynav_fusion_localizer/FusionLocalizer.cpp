#include "easynav_fusion_localizer/FusionLocalizer.hpp"

#include "easynav_localizer/LocalizerNode.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

#include <GeographicLib/UTMUPS.hpp>

#include "easynav_common/YTSession.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace easynav
{

void FusionLocalizer::on_initialize()
{

  try {

    auto node = get_node();

    // Subscribe to initial pose
    init_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&FusionLocalizer::init_pose_callback, this, std::placeholders::_1));

    auto localizer_node = std::dynamic_pointer_cast<LocalizerNode>(node);

    const std::string & plugin_name = this->get_plugin_name();
    const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

    RCLCPP_INFO(localizer_node->get_logger(), "Using tf_prefix: '%s'", tf_info.tf_prefix.c_str());
    RCLCPP_INFO(localizer_node->get_logger(), "Using parameter namespace: '%s'",
    plugin_name.c_str());

    // Detect which filters have parameters configured by checking
    // parameter overrides (loaded from YAML before declaration)
    const std::string global_prefix = plugin_name + ".global_filter.";
    const std::string local_prefix = plugin_name + ".local_filter.";

    const auto & overrides =
      localizer_node->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [key, _] : overrides) {
      if (!has_global_filter_ && key.rfind(global_prefix, 0) == 0) {
        has_global_filter_ = true;
      }
      if (!has_local_filter_ && key.rfind(local_prefix, 0) == 0) {
        has_local_filter_ = true;
      }
      if (has_global_filter_ && has_local_filter_) {
        break;
      }
    }

    if (!has_global_filter_ && !has_local_filter_) {
      RCLCPP_FATAL(
        localizer_node->get_logger(),
        "No parameters found for either '%s' or '%s'. "
        "At least one filter must be configured.",
        global_prefix.c_str(), local_prefix.c_str());
      throw std::runtime_error(
        "FusionLocalizer: no global_filter or local_filter parameters detected. "
        "At least one filter must be configured.");
    }

    if (has_global_filter_) {
      ukf_global_ = std::make_unique<robot_localization::UkfWrapper>(
        localizer_node, tf_info.tf_prefix, plugin_name + ".global_filter", false
      );
      ukf_global_->initialize();
    } else {
      RCLCPP_WARN(localizer_node->get_logger(),
          "No global_filter parameters found. Global filter will NOT be created.");
    }

    if (has_local_filter_) {
      ukf_local_ = std::make_unique<robot_localization::UkfWrapper>(
        localizer_node, tf_info.tf_prefix, plugin_name + ".local_filter", true
      );
      ukf_local_->initialize();
    } else {
      RCLCPP_WARN(localizer_node->get_logger(),
          "No local_filter parameters found. Local filter will NOT be created.");
    }

    // GPS-related setup only needed when global filter is active
    if (has_global_filter_) {
      localizer_node->declare_parameter(plugin_name + ".latitude_origin", double(0.0));
      localizer_node->get_parameter(plugin_name + ".latitude_origin", latitude_origin_);

      localizer_node->declare_parameter(plugin_name + ".longitude_origin", double(0.0));
      localizer_node->get_parameter(plugin_name + ".longitude_origin", longitude_origin_);

      localizer_node->declare_parameter(plugin_name + ".altitude_origin", double(0.0));
      localizer_node->get_parameter(plugin_name + ".altitude_origin", altitude_origin_);

      localizer_node->declare_parameter(
        plugin_name + ".navsatfix_topic", std::string("gps/filtered"));
      localizer_node->get_parameter(plugin_name + ".navsatfix_topic", navsatfix_topic_);
      navsat_pub_ = localizer_node->create_publisher<sensor_msgs::msg::NavSatFix>(
        navsatfix_topic_, rclcpp::QoS(10));
    }

  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "Critical failure initializing UkfWrapper: %s",
      e.what());
    throw std::runtime_error(std::string("Failed to initialize UkfWrapper: ") + e.what());
  }

  if (has_global_filter_) {
    GeographicLib::UTMUPS::Forward(latitude_origin_, longitude_origin_, UTM_zone_number_,
        UTM_zone_northp_, UTM_origin_x_, UTM_origin_y_);
    UTM_zone_ = std::to_string(UTM_zone_number_) + (UTM_zone_northp_ ? "N" : "S");
    UTM_origin_z_ = altitude_origin_;

    n_gps_sensors_ = static_cast<int>(ukf_global_->getGpsCallbackDataArr().size());
    last_gps_stamp_.resize(n_gps_sensors_, rclcpp::Time(0, 0, RCL_ROS_TIME));
  }

  RCLCPP_INFO(get_node()->get_logger(), "FusionLocalizer (UKF) initialized successfully.");
}

void FusionLocalizer::init_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (has_global_filter_) {
    nav_msgs::msg::Odometry global_odom;
    // Get current position to calculate the offset
    if (ukf_global_->getFilteredOdometryMessage(&global_odom)) {
      double current_x = global_odom.pose.pose.position.x;
      double current_y = global_odom.pose.pose.position.y;
      
      // Shift the UTM origin so that the current GPS position will now map to msg's position
      UTM_origin_x_ += (current_x - msg->pose.pose.position.x);
      UTM_origin_y_ += (current_y - msg->pose.pose.position.y);
      
      RCLCPP_INFO(get_node()->get_logger(), 
        "Initial pose reset UTM origin. Shifted X by %.2f, Y by %.2f to match pose (%.2f, %.2f)",
        (current_x - msg->pose.pose.position.x), (current_y - msg->pose.pose.position.y),
        msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
    
    // Forward initial pose to global filter to make the state jump immediately
    ukf_global_->setPoseCallback(msg);
    first_pose_received_ = true;
  }
  
  if (has_local_filter_) {
    ukf_local_->setPoseCallback(msg);
  }
}

void FusionLocalizer::update_rt(NavState & nav_state)
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  if (has_global_filter_) {
    if (n_gps_sensors_ && nav_state.has_group("gnss")) {
      auto gps_data = nav_state.get_group<easynav::GNSSPerception>(std::string("gnss"));
      const auto & gps_cb_arr = ukf_global_->getGpsCallbackDataArr();
      for (int i = 0; i < n_gps_sensors_; ++i) {
        if (gps_data[i]->data.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
          continue;
        }
        rclcpp::Time gps_time(gps_data[i]->data.header.stamp);
        if (gps_time > last_gps_stamp_[i]) {
          EASYNAV_TRACE_NAMED_EVENT("fusion_localizer_process_gps");
          last_gps_stamp_[i] = gps_time;
          auto pose =
            std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(navsatfix_to_pose(
              gps_data[i]->data));
          if (!first_pose_received_) {
            RCLCPP_INFO(get_node()->get_logger(),
                "First valid GPS fix received. Initializing filter state.");
            if (nav_state.has("imu")) {
              auto imu_data = nav_state.get<IMUPerception>(std::string("imu"));
              pose->pose.pose.orientation = imu_data.data.orientation;
            }
            ukf_global_->setPoseCallback(pose);
            first_pose_received_ = true;
            continue;
          }
          ukf_global_->poseCallback(
            pose,
            gps_cb_arr[i],
            tf_info.map_frame,
            gps_data[i]->data.header.frame_id,
            false
          );
        }
      }
    }

    ukf_global_->periodicUpdate();

    nav_msgs::msg::Odometry global_odom;
    if (ukf_global_->getFilteredOdometryMessage(&global_odom)) {
      nav_state.set("robot_pose", global_odom);
      navsat_pub_->publish(odom_to_navsatfix(global_odom));
    }
  }

  if (has_local_filter_) {
    ukf_local_->periodicUpdate();

    nav_msgs::msg::Odometry local_odom;
    if (ukf_local_->getFilteredOdometryMessage(&local_odom)) {
      nav_state.set("robot_pose_local", local_odom);
    }
  }
}

void FusionLocalizer::update([[maybe_unused]] NavState & nav_state)
{

}

geometry_msgs::msg::PoseWithCovarianceStamped FusionLocalizer::navsatfix_to_pose(
  const sensor_msgs::msg::NavSatFix & navsat_msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

  pose_msg.header = navsat_msg.header;

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  pose_msg.header.frame_id = tf_info.map_frame;

  double utm_x, utm_y;
  int zone;
  bool northp;

  GeographicLib::UTMUPS::Forward(
    navsat_msg.latitude,
    navsat_msg.longitude,
    zone,
    northp,
    utm_x,
    utm_y,
    UTM_zone_number_);

  pose_msg.pose.pose.position.x = utm_x - UTM_origin_x_;
  pose_msg.pose.pose.position.y = utm_y - UTM_origin_y_;
  pose_msg.pose.pose.position.z = navsat_msg.altitude - UTM_origin_z_;

  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = 0.0;
  pose_msg.pose.pose.orientation.w = 1.0;

  pose_msg.pose.covariance.fill(0.0);

  double default_var = 1.0; // 1 meter variance standard

  if (navsat_msg.position_covariance_type ==
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN)
  {
    pose_msg.pose.covariance[0] = navsat_msg.position_covariance[0];
    pose_msg.pose.covariance[7] = navsat_msg.position_covariance[4];
    pose_msg.pose.covariance[14] = navsat_msg.position_covariance[8];

    pose_msg.pose.covariance[21] = default_var;
    pose_msg.pose.covariance[28] = default_var;
    pose_msg.pose.covariance[35] = default_var;
  } else {
      // Fallback variances if GPS doesn't provide them
    pose_msg.pose.covariance[0] = default_var;
    pose_msg.pose.covariance[7] = default_var;
    pose_msg.pose.covariance[14] = default_var;

    pose_msg.pose.covariance[21] = default_var;
    pose_msg.pose.covariance[28] = default_var;
    pose_msg.pose.covariance[35] = default_var;

    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "NavSatFix covariance type unknown or invalid. Using default covariance.");
  }

  return pose_msg;
}

sensor_msgs::msg::NavSatFix FusionLocalizer::odom_to_navsatfix(
  const nav_msgs::msg::Odometry & odom_msg)
{
  sensor_msgs::msg::NavSatFix navsat_msg;

  navsat_msg.header = odom_msg.header;
  navsat_msg.header.frame_id = "gps_link";
  navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  navsat_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  const double utm_x = odom_msg.pose.pose.position.x + UTM_origin_x_;
  const double utm_y = odom_msg.pose.pose.position.y + UTM_origin_y_;

  double latitude = 0.0;
  double longitude = 0.0;
  GeographicLib::UTMUPS::Reverse(
    UTM_zone_number_, UTM_zone_northp_, utm_x, utm_y, latitude, longitude);

  navsat_msg.latitude = latitude;
  navsat_msg.longitude = longitude;
  navsat_msg.altitude = odom_msg.pose.pose.position.z + UTM_origin_z_;

  navsat_msg.position_covariance[0] = odom_msg.pose.covariance[0];
  navsat_msg.position_covariance[4] = odom_msg.pose.covariance[7];
  navsat_msg.position_covariance[8] = odom_msg.pose.covariance[14];

  navsat_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return navsat_msg;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::FusionLocalizer, easynav::LocalizerMethodBase)
