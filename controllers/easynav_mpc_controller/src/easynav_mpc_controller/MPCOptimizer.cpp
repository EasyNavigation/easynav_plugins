#include "easynav_mpc_controller/MPCOptimizer.hpp"

namespace easynav
{

MPCParameters::MPCParameters(Eigen::Vector2d goal,
  Eigen::Vector3d x0,
  Eigen::Vector3d theta0,
  const pcl::PointCloud<pcl::PointXYZ> & points,
  int N,
  double dt)
  :goal(goal), x0(x0), theta0(theta0), points(points), N_(N), dt_(dt) {}

MPCParameters::~MPCParameters() = default;

int MPCParameters::get_steps()
{
  return N_;
}

double MPCParameters::get_timestep()
{
  return dt_;
}

double MPCParameters::get_angular_tracking_cost()
{
  return qtheta_;
}

Eigen::Matrix2d MPCParameters::get_effort_cost()
{
  return R_;
}

Eigen::Matrix2d MPCParameters::get_tracking_cost()
{
  return Q_;
}

Eigen::Matrix2d MPCParameters::get_smooth_cost()
{
  return Rd_;
}

MPCOptimizer::MPCOptimizer() {}

MPCOptimizer::~MPCOptimizer() = default;

Eigen::Vector3d
MPCOptimizer::kinematic_model(const Eigen::Vector3d & x, const Eigen::Vector3d & q, double v, double w, double dt)
{
  Eigen::Vector3d x_k1;
  x_k1[0] = x[0] + v * cos(q[2]) * dt;
  x_k1[1] = x[1] + v * sin(q[2]) * dt;
  x_k1[2] = q[2] + w * dt;
  return x_k1;
}

double
MPCOptimizer::cost_function(const std::vector<double> & u, std::vector<double> & grad, void *data)
{
  MPCParameters *params = reinterpret_cast<MPCParameters *>(data);

  Eigen::Vector3d position = params->x0;
  Eigen::Vector3d orientation = params->theta0;
  Eigen::Vector3d state;
  int N = params->get_steps();
  double dt = params->get_timestep();
  double qtheta = params->get_angular_tracking_cost();
  double cost = 0.0;
  double penalty = 0.25;

  Eigen::Matrix2d R = params->get_effort_cost();
  Eigen::Matrix2d Q = params->get_tracking_cost();
  Eigen::Matrix2d Rd = params->get_smooth_cost();

  for (int i = 0; i < N; ++i) {
    double v = u[2 * i];
    double w = u[2 * i + 1];
    double dv, dw;
    if(i < (N - 1)) {
      dv = u[2 * (i + 1)] - u[2 * i];
      dw = u[2 * (i + 1) + 1] - u[2 * i + 1];
    } else {
      dv = 0.0;
      dw = 0.0;
    }

    state = MPCOptimizer::kinematic_model(position, orientation, v, w, dt);

    Eigen::Vector2d pos = state.head<2>();
    Eigen::Vector2d error = params->goal - pos;
    double error_theta = (atan2((error[1]), (error[0]))) - state[2];
    Eigen::Vector2d uk(v, w);
    Eigen::Vector2d duk(dv, dw);

    for (const auto & point : params->points) {
    // double min_obs_dist = std::numeric_limits<double>::max();
    // for (const auto &[x, y] : trajectory) {
    //   double dx = point.x - x;
    //   double dy = point.y - y;
    //   double dist = std::hypot(dx, dy);
    //   if (dist < min_obs_dist) {min_obs_dist = dist;}
    // }
    // min_obs_overall = std::min(min_obs_overall, min_obs_dist);

    // // Safety margin (robot radius + margin)
    // if (min_obs_dist < safety_radius_) {
    //   // Heavy penalty for collision risk
    //   cost += 5000.0 * std::pow(safety_radius_ - min_obs_dist, 2) * (1.0 + v);
    // } else {
    //   // Small penalty: encourage keeping clearance
    //   cost += 1.0 / (min_obs_dist * min_obs_dist);
      double dist = std::hypot(point.x - pos[0] , point.y -pos[1]);
      if (dist < 1.0){
        cost+=penalty * std::pow(1.0 - dist, 2) * std::pow(v, 2);
        //cost+=penalty;
      }
    }

    // Tracking cost
    cost += Q(0, 0) * error[0] * error[0] + Q(1,
      1) * error[1] * error[1] + qtheta * error_theta * error_theta;
    // Effort Cost
    cost += R(0, 0) * v * v + R(1, 1) * w * w;
    // Smooth Cost
    cost += Rd(0, 0) * dv * dv + Rd(1, 1) * dw * dw;
  }

  return cost;

}

double MPCOptimizer::nlopt_cost_callback(
  const std::vector<double> & x,
  std::vector<double> & grad,
  void *data)
{
  auto *cbdata = static_cast<NLoptCallbackData*>(data);
  return cbdata->optimizer->cost_function(x, grad, cbdata->params);
}

}  // namespace easynav
