# Include after acg138_map_core has been declared; NAV2 and runtime stay unchanged.
target_sources(acg138_map_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/nav3_wire.cpp)
if(BUILD_TESTING)
  enable_language(C)
  enable_testing()
  # Frozen reference is linked ONLY into the offline test, never the ROS node.
  add_library(nav3_golden_oracle STATIC tests/reference/nav3/nav_exec_wire.c)
  target_include_directories(nav3_golden_oracle PUBLIC tests/reference/nav3)
  add_executable(test_nav3_wire tests/test_nav3_wire.cpp)
  target_compile_options(test_nav3_wire PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_nav3_wire PRIVATE acg138_map_core nav3_golden_oracle)
  add_test(NAME nav3_wire_golden COMMAND test_nav3_wire ${CMAKE_CURRENT_BINARY_DIR}/nav3-test-results)
endif()
