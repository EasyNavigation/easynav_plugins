// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief A quadratic program solved with OSQP (0.6 or 1.0, whichever osqp_vendor brings).

#ifndef EASYNAV_MPC_CONTROLLER__QPSOLVER_HPP_
#define EASYNAV_MPC_CONTROLLER__QPSOLVER_HPP_

#include "Eigen/Core"
#include "Eigen/SparseCore"

namespace easynav
{
namespace mpc
{

/// @brief min 0.5 x' P x + q' x  subject to  l <= A x <= u. P symmetric positive semidefinite
/// (only its upper triangle is read).
struct QPProblem
{
  Eigen::SparseMatrix<double> P;
  Eigen::VectorXd q;
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
};

enum class QPStatus
{
  Solved,          ///< Within the tolerances.
  Inaccurate,      ///< Solved to a lower accuracy (or out of iterations): usable.
  Infeasible,      ///< The constraints cannot all hold.
  Failed           ///< Setup error, non-finite data or any other status.
};

struct QPSettings
{
  int max_iterations {400};  ///< Bounds the solve time.
  double eps_abs {1e-3};
  double eps_rel {1e-3};
};

/// @brief Solves \p problem; \p x is the solution if the status is Solved or Inaccurate.
QPStatus solve_qp(const QPProblem & problem, const QPSettings & settings, Eigen::VectorXd & x);

}  // namespace mpc
}  // namespace easynav

#endif  // EASYNAV_MPC_CONTROLLER__QPSOLVER_HPP_
