if(BUILD_TESTING)
  add_library(nav3_exec_oracle STATIC
    tests/reference/nav3/candidate26/softcore/firmware/nav_exec.c
    tests/reference/nav3/candidate26/softcore/firmware/nav_protocol.c
    tests/reference/nav3/candidate26/softcore/firmware/astar.c)
  add_executable(test_nav3_map tests/test_nav3_map.cpp)
  target_compile_options(test_nav3_map PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_nav3_map PRIVATE acg138_map_core nav3_exec_oracle nav3_guard_oracle nav3_golden_oracle)
  add_test(NAME nav3_map_upload COMMAND test_nav3_map ${CMAKE_CURRENT_BINARY_DIR}/nav3-map-results)
endif()
