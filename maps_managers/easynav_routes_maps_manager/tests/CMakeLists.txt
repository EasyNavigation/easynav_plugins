find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(routes_mapsmanager_tests
  routes_mapsmanager_tests.cpp
)

target_link_libraries(routes_mapsmanager_tests
  ${PROJECT_NAME}
  easynav_common::easynav_common
  easynav_core::easynav_core
)
