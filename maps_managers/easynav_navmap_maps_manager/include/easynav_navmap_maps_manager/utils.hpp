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

#ifndef EASYNAV_NAVMAP_MAPS_UTILS_HPP_
#define EASYNAV_NAVMAP_MAPS_UTILS_HPP_

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


inline void triangle_angles_deg(
  const Eigen::Vector3f & A,
  const Eigen::Vector3f & B,
  const Eigen::Vector3f & C,
  float & angA, float & angB, float & angC);

// PCA local para obtener plano tangente en v
// Devuelve dos ejes ortogonales t1, t2 en el plano tangente.
// Si falla PCA (muy pocos puntos), usa un par ortonormal arbitrario.
inline void local_tangent_basis(
  const std::vector<Eigen::Vector3f> & nbrs,
  Eigen::Vector3f & t1, Eigen::Vector3f & t2);

// Ordena vecinos por ángulo en el plano tangente de v
inline void sort_neighbors_angular(
  const Eigen::Vector3f & vpos,
  const std::vector<std::pair<int, Eigen::Vector3f>> & nbrs,  // (idx, pos)
  std::vector<int> & out_ordered);

// Intenta crear un triángulo (i,j,k) bajo las restricciones
inline bool try_add_triangle(
  int i, int j, int k,
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const Params & P,
  std::unordered_set<TriKey, TriHasher> & tri_set,
  std::unordered_set<EdgeKey, EdgeHasher> & edge_set,
  std::vector<Triangle> & tris);


std::vector<Triangle> grow_surface_from_seed(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  int seed_idx,
  const Params & P);
}  // namespace navmap
}  // namespace easynav
#endif  // EASYNAV_NAVMAP_MAPS_UTILS_HPP_
