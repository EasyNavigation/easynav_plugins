ament_add_gtest(costmap_bresenham_2d costmap_bresenham_2d.cpp)
target_link_libraries(costmap_bresenham_2d ${PROJECT_NAME})

ament_add_gtest(costmap_2d_tests costmap_2d_tests.cpp)
target_link_libraries(costmap_2d_tests ${PROJECT_NAME})
