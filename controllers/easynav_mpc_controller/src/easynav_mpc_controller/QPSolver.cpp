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

#include "easynav_mpc_controller/QPSolver.hpp"

#include <osqp.h>

#include <algorithm>
#include <vector>

// OSQP 1.0 (Lyrical, Rolling) defines OSQP_API; 0.6 (Humble, Jazzy, Kilted) does not, and names
// its types, solver and matrices differently.
#ifdef OSQP_API
using QPInt = OSQPInt;
using QPFloat = OSQPFloat;
#else
using QPInt = c_int;
using QPFloat = c_float;
#endif

namespace easynav
{
namespace mpc
{

namespace
{

/// A matrix in the compressed sparse column arrays OSQP takes (it does not copy them).
struct Csc
{
  explicit Csc(const Eigen::SparseMatrix<double> & m)
  : rows(m.rows()), cols(m.cols())
  {
    Eigen::SparseMatrix<double> c = m;
    c.makeCompressed();
    x.assign(c.valuePtr(), c.valuePtr() + c.nonZeros());
    i.assign(c.innerIndexPtr(), c.innerIndexPtr() + c.nonZeros());
    p.assign(c.outerIndexPtr(), c.outerIndexPtr() + c.cols() + 1);
  }

  QPInt rows, cols;
  std::vector<QPFloat> x;
  std::vector<QPInt> i, p;
};

std::vector<QPFloat> to_vector(const Eigen::VectorXd & v)
{
  return std::vector<QPFloat>(v.data(), v.data() + v.size());
}

QPStatus status_of(QPInt status)
{
  switch (status) {
    case OSQP_SOLVED:
      return QPStatus::Solved;
    case OSQP_SOLVED_INACCURATE:
    case OSQP_MAX_ITER_REACHED:
      return QPStatus::Inaccurate;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
      return QPStatus::Infeasible;
    default:
      return QPStatus::Failed;
  }
}

}  // namespace

QPStatus solve_qp(const QPProblem & problem, const QPSettings & settings, Eigen::VectorXd & x)
{
  const QPInt n = problem.q.size();
  const QPInt m = problem.l.size();
  if (n == 0 || problem.P.rows() != n || problem.A.cols() != n || problem.A.rows() != m ||
    problem.u.size() != m || !problem.q.allFinite() || problem.l.hasNaN() ||
    problem.u.hasNaN())
  {
    return QPStatus::Failed;
  }

  Csc P(Eigen::SparseMatrix<double>(problem.P.triangularView<Eigen::Upper>()));
  Csc A(problem.A);
  auto q = to_vector(problem.q);
  auto l = to_vector(problem.l);
  auto u = to_vector(problem.u);
  // OSQP's infinity is 1e30
  for (QPInt k = 0; k < m; ++k) {
    l[k] = std::max<QPFloat>(l[k], -1e30);
    u[k] = std::min<QPFloat>(u[k], 1e30);
  }

#ifdef OSQP_API
  OSQPSettings s;
  osqp_set_default_settings(&s);
  s.verbose = 0;
  s.polishing = 0;
  s.max_iter = settings.max_iterations;
  s.eps_abs = settings.eps_abs;
  s.eps_rel = settings.eps_rel;

  OSQPCscMatrix Pm {}, Am {};
  OSQPCscMatrix_set_data(&Pm, n, n, P.x.size(), P.x.data(), P.i.data(), P.p.data());
  OSQPCscMatrix_set_data(&Am, m, n, A.x.size(), A.x.data(), A.i.data(), A.p.data());

  OSQPSolver * solver = nullptr;
  if (osqp_setup(&solver, &Pm, q.data(), &Am, l.data(), u.data(), m, n, &s) != 0 || !solver) {
    osqp_cleanup(solver);
    return QPStatus::Failed;
  }
  osqp_solve(solver);
  const auto status = status_of(solver->info->status_val);
  if (status == QPStatus::Solved || status == QPStatus::Inaccurate) {
    x = Eigen::Map<const Eigen::Matrix<QPFloat, Eigen::Dynamic, 1>>(solver->solution->x, n)
      .cast<double>();
  }
  osqp_cleanup(solver);
#else
  OSQPSettings s;
  osqp_set_default_settings(&s);
  s.verbose = 0;
  s.polish = 0;
  s.max_iter = settings.max_iterations;
  s.eps_abs = settings.eps_abs;
  s.eps_rel = settings.eps_rel;

  csc Pm {static_cast<c_int>(P.x.size()), n, n, P.p.data(), P.i.data(), P.x.data(), -1};
  csc Am {static_cast<c_int>(A.x.size()), m, n, A.p.data(), A.i.data(), A.x.data(), -1};
  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = &Pm;
  data.A = &Am;
  data.q = q.data();
  data.l = l.data();
  data.u = u.data();

  OSQPWorkspace * work = nullptr;
  if (osqp_setup(&work, &data, &s) != 0 || !work) {
    osqp_cleanup(work);
    return QPStatus::Failed;
  }
  osqp_solve(work);
  const auto status = status_of(work->info->status_val);
  if (status == QPStatus::Solved || status == QPStatus::Inaccurate) {
    x = Eigen::Map<const Eigen::Matrix<QPFloat, Eigen::Dynamic, 1>>(work->solution->x, n)
      .cast<double>();
  }
  osqp_cleanup(work);
#endif
  if ((status == QPStatus::Solved || status == QPStatus::Inaccurate) && !x.allFinite()) {
    return QPStatus::Failed;
  }
  return status;
}

}  // namespace mpc
}  // namespace easynav
