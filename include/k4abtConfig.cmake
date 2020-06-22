include(CMakeFindDependencyMacro)

find_dependency(k4a 1.3.0 REQUIRED)

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/k4abtTargets.cmake")