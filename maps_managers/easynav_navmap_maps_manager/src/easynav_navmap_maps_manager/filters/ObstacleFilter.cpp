// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include <expected>
#include <string>

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "navmap_core/NavMap.hpp"

#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{
namespace navmap
{

ObstacleFilter::ObstacleFilter()
{

}

std::expected<void, std::string>
ObstacleFilter::on_initialize()
{
  return {};
}

void ObstacleFilter::update(::easynav::NavState & nav_state)
{
  auto t0 = parent_node_->now();
  std::cerr << "ObstacleFilter::update" << std::endl;

  if (!nav_state.has("map.navmap")) return;
  if (!nav_state.has("points"))     return;

  const auto & perceptions = nav_state.get<PointPerceptions>("points");
  navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");

  // Si necesitas limpieza total en cada ciclo, mantén esto:
  navmap_.layer_clear<float>(get_layer_name(), 0.0f);

  auto t1 = parent_node_->now();

  auto fused = PointPerceptionsOpsView(perceptions)
                 .downsample(0.3)
                 .fuse(get_tf_prefix() + "map")
                 ->filter({-5.0, -5.0, NAN}, {5.0, 5.0, NAN})
                 .as_points();

  // (Opcional pero muy rentable) Orden espacial para mejorar locality del hint:
  // ordenar por una clave Morton simple 2D (x,y). Implementación ligera:
  auto morton2D = [](float x, float y) -> uint64_t {
    // normaliza a rejilla 1 cm en [-64,64] m => 12800 pasos (cambia a tu rango)
    auto q = [](float v){
      int32_t iv = static_cast<int32_t>(std::lrintf((v + 64.f) * 100.f));
      if (iv < 0) iv = 0; else if (iv > 12800) iv = 12800;
      return static_cast<uint32_t>(iv);
    };
    auto part = [](uint32_t v){
      uint64_t x = v;
      x = (x | (x << 16)) & 0x0000FFFF0000FFFFULL;
      x = (x | (x << 8 )) & 0x00FF00FF00FF00FFULL;
      x = (x | (x << 4 )) & 0x0F0F0F0F0F0F0F0FULL;
      x = (x | (x << 2 )) & 0x3333333333333333ULL;
      x = (x | (x << 1 )) & 0x5555555555555555ULL;
      return x;
    };
    uint64_t xi = part(q(x));
    uint64_t yi = part(q(y)) << 1;
    return xi | yi;
  };

  std::sort(fused.begin(), fused.end(),
            [&](const auto &a, const auto &b){
              return morton2D(a.x, a.y) < morton2D(b.x, b.y);
            });

  auto t2 = parent_node_->now();

  // Acumulación de h máximos por celda (reduce layer_set)
  struct NavCelIdHash {
    std::size_t operator()(const ::navmap::NavCelId &c) const noexcept {
      // Si NavCelId ya es entero/struct con hash, usa ese. Si no, adapta esto.
      return std::hash<uint64_t>{}(static_cast<uint64_t>(c));
    }
  };
  std::unordered_map<::navmap::NavCelId, float, NavCelIdHash> cell_max;
  cell_max.reserve(fused.size()); // heurística

  size_t sidx = 0;                             // deja que locate lo actualice
  std::optional<::navmap::NavCelId> last_cid;  // hint
  ::navmap::NavMap::LocateOpts opts;           // reutilizado
  Eigen::Vector3f bary;                        // reutilizado
  Eigen::Vector3f hit;                         // reutilizado

  // Cota inferior rápida (si procede). Ajusta a tu nivel de suelo esperado.
  constexpr float kMinAboveGround = 0.0f;
  constexpr float kWriteThreshold = 0.1f;

  auto t3 = parent_node_->now();

  for (const auto & p : fused) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    // Descarte barato: si ya sabes que p.z está por debajo del suelo esperado, sáltalo.
    if (p.z < kMinAboveGround) continue;

    ::navmap::NavCelId cid;
    bool located = false;

    // intento con hint
    if (last_cid) {
      opts.hint_cid = *last_cid;
      located = navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit, opts);
    } else {
      // evita basura en opts.hint_cid si el API la lee sin comprobar
      opts = ::navmap::NavMap::LocateOpts{};
      located = navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit, opts);
    }

    if (!located) {
      // fallback sin hint
      opts = ::navmap::NavMap::LocateOpts{};
      located = navmap_.locate_navcel({p.x, p.y, p.z}, sidx, cid, bary, &hit, opts);
      if (!located) continue;
    }

    last_cid = cid;

    const float h = static_cast<float>(p.z) - hit.z();
    if (!(h > kWriteThreshold)) continue;  // incluye NaN/inf y <= threshold

    auto it = cell_max.find(cid);
    if (it == cell_max.end()) {
      cell_max.emplace(cid, h);
    } else if (h > it->second) {
      it->second = h;
    }
  }

  // Aplicar a la capa en bloque
  for (const auto & kv : cell_max) {
    navmap_.layer_set<float>(get_layer_name(), kv.first, kv.second);
  }

  auto t4 = parent_node_->now();
  nav_state.set("map.navmap", navmap_);
  auto t5 = parent_node_->now();

  std::cerr << "t1 = " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << std::endl;
  std::cerr << "t2 = " << std::fixed << std::setprecision(10) << (t2 - t1).seconds() << std::endl;
  std::cerr << "t3 = " << std::fixed << std::setprecision(10) << (t3 - t2).seconds() << std::endl;
  std::cerr << "t4 = " << std::fixed << std::setprecision(10) << (t4 - t3).seconds() << std::endl;
  std::cerr << "t5 = " << std::fixed << std::setprecision(10) << (t5 - t4).seconds() << std::endl;
}


}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
