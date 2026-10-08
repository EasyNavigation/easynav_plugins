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
/// \brief Includes octomap/octomap.h without its <ciso646> C++20 #warning.

#ifndef EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_HPP_
#define EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_HPP_

// octomap/OcTreeKey.h includes <ciso646>, deprecated since C++20 (GCC >= 15 warns)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "octomap/octomap.h"
#pragma GCC diagnostic pop

#endif  // EASYNAV_OCTOMAP_MAPS_MANAGER__OCTOMAP_HPP_
