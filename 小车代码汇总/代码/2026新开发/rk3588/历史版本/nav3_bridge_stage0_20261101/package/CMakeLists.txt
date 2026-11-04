# Standalone offline harness ONLY. Never overwrite the board package CMakeLists.
add_library(acg138_map_core STATIC)
target_include_directories(acg138_map_core PUBLIC include)
target_compile_options(acg138_map_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
include(cmake/Nav3Stage0.cmake)
