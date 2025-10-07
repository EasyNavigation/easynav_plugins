// Copyright (c) 2018 Intel Corporation
//
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

/* OccupancyGrid map input-output library */

#include "easynav_navmap_maps_manager/utils.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace easynav
{
namespace navmap
{

// PCA local para obtener plano tangente en v
// Devuelve dos ejes ortogonales t1, t2 en el plano tangente.
// Si falla PCA (muy pocos puntos), usa un par ortonormal arbitrario.
inline void local_tangent_basis(
  const std::vector<Eigen::Vector3f> & nbrs,
  Eigen::Vector3f & t1, Eigen::Vector3f & t2)
{
  if (nbrs.size() < 3) {
    // base por defecto (proyección estable)
    t1 = Eigen::Vector3f::UnitX();
    t2 = Eigen::Vector3f::UnitY();
    return;
  }

  // Centra
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (auto & p : nbrs) {
    mean += p;
  }
  mean /= static_cast<float>(nbrs.size());

  // Covarianza
  Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
  for (auto & p : nbrs) {
    Eigen::Vector3f d = p - mean;
    C += d * d.transpose();
  }
  C /= static_cast<float>(nbrs.size());

  // Autovectores
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
  // Valores ascendentes: 0 -> menor (normal aproximada)
  // Eigen::Vector3f e0 = es.eigenvectors().col(0); // normal aproximada
  Eigen::Vector3f e1 = es.eigenvectors().col(1);
  Eigen::Vector3f e2 = es.eigenvectors().col(2);

  // Plano tangente: proyectaremos en {e1, e2}
  // Asegura ortonormalidad y evita inversión esporádica
  t1 = e1.normalized();
  t2 = (e2 - e2.dot(t1) * t1).normalized();
}

inline void triangle_angles_deg(
  const Eigen::Vector3f & A,
  const Eigen::Vector3f & B,
  const Eigen::Vector3f & C,
  float & angA, float & angB, float & angC)
{
  // Lados opuestos a A, B, C
  float a = (B - C).norm();
  float b = (C - A).norm();
  float c = (A - B).norm();

  // Evitar divisiones por cero
  const float eps = 1e-12f;
  a = std::max(a, eps);
  b = std::max(b, eps);
  c = std::max(c, eps);

  // Ley de cosenos con clamp numérico
  auto angle_from = [](float opp, float x, float y) -> float {
      float cosv = (x * x + y * y - opp * opp) / (2.0f * x * y);
      cosv = std::min(1.0f, std::max(-1.0f, cosv));
      return std::acos(cosv) * 180.0f / static_cast<float>(M_PI);
    };

  angA = angle_from(a, b, c);
  angB = angle_from(b, c, a);
  angC = angle_from(c, a, b);
}

// Ordena vecinos por ángulo en el plano tangente de v
inline void sort_neighbors_angular(
  const Eigen::Vector3f & vpos,
  const std::vector<std::pair<int, Eigen::Vector3f>> & nbrs,  // (idx, pos)
  std::vector<int> & out_ordered)
{
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(nbrs.size() + 1);
  pts.push_back(vpos);
  for (auto & it : nbrs) {
    pts.push_back(it.second);
  }

  Eigen::Vector3f t1, t2;
  local_tangent_basis(pts, t1, t2);

  std::vector<std::pair<float, int>> ang_idx;
  ang_idx.reserve(nbrs.size());
  for (auto & it : nbrs) {
    Eigen::Vector3f d = it.second - vpos;
    float x = d.dot(t1);
    float y = d.dot(t2);
    float ang = std::atan2(y, x); // -pi..pi
    ang_idx.emplace_back(ang, it.first);
  }

  std::sort(ang_idx.begin(), ang_idx.end(),
    [](auto & a, auto & b){return a.first < b.first;});

  out_ordered.clear();
  out_ordered.reserve(ang_idx.size());
  for (auto & ai : ang_idx) {
    out_ordered.push_back(ai.second);
  }
}

// Intenta crear un triángulo (i,j,k) bajo las restricciones
inline bool try_add_triangle(
  int i, int j, int k,
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const Params & P,
  std::unordered_set<TriKey, TriHasher> & tri_set,
  std::unordered_set<EdgeKey, EdgeHasher> & edge_set,
  std::vector<Triangle> & tris)
{
  TriKey tk = make_tri(i, j, k);
  if (tri_set.find(tk) != tri_set.end()) {return false;}

  const auto & A = cloud[i];
  const auto & B = cloud[j];
  const auto & C = cloud[k];
  if (!pcl::isFinite(A) || !pcl::isFinite(B) || !pcl::isFinite(C)) {return false;}

  // Longitudes máximas de arista
  auto dAB = dist3(A, B);
  auto dBC = dist3(B, C);
  auto dCA = dist3(C, A);
  if (dAB > P.max_edge_len || dBC > P.max_edge_len || dCA > P.max_edge_len) {return false;}

  Eigen::Vector3f a(A.x, A.y, A.z);
  Eigen::Vector3f b(B.x, B.y, B.z);
  Eigen::Vector3f c(C.x, C.y, C.z);

  // Área mínima
  if (tri_area(a, b, c) < P.min_area) {return false;}

  // Pendiente máxima
  float slope = tri_slope_deg(a, b, c);
  if (slope > P.max_slope_deg) {return false;}

  // Ángulo mínimo en los tres vértices
  float angA, angB, angC;
  triangle_angles_deg(a, b, c, angA, angB, angC);
  if (angA < P.min_angle_deg || angB < P.min_angle_deg || angC < P.min_angle_deg) {
    return false;
  }

  {
    const Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f n = (b - a).cross(c - a);
    if (n.norm() < 1e-9f) {return false;}
    if (n.dot(up) < 0.0f) {
      std::swap(j, k);
      // si más abajo usas b/c otra vez, re-asigna:
      // b = Eigen::Vector3f(cloud[j].x, cloud[j].y, cloud[j].z);
      // c = Eigen::Vector3f(cloud[k].x, cloud[k].y, cloud[k].z);
    }
  }

  // Aceptar
  tri_set.insert(tk);
  edge_set.insert(make_edge(i, j));
  edge_set.insert(make_edge(j, k));
  edge_set.insert(make_edge(k, i));
  tris.emplace_back(Triangle(i, j, k));
  return true;
}


}  // namespace navmap
}  // namespace easynav
