#include "easynav_mpc_controller/MPCOptimizer.hpp"

namespace easynav
{

MPCParameters::MPCParameters(Eigen::Vector2d goal,
  Eigen::Vector3d x0,
  Eigen::Vector3d theta0,
  Eigen::Matrix2d Q,
  Eigen::Matrix2d R,
  Eigen::Matrix2d Rd,
  double qtheta,
  int N,
  double dt,
  const pcl::PointCloud<pcl::PointXYZ> & points)
  :x0(x0), theta0(theta0), Q(Q), R(R), Rd(Rd), qtheta(qtheta), N(N),
    dt(dt), points(points) {}

MPCParameters::~MPCParameters() = default;

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
  int N = params->N;
  double dt = params->dt;
  double qtheta = params->qtheta;
  double cost = 0.0;

  Eigen::Matrix2d R = params->R;
  Eigen::Matrix2d Q = params->Q;
  Eigen::Matrix2d Rd = params->Rd;

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
