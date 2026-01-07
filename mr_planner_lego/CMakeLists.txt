cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(mr_planner_lego VERSION 0.1.0 LANGUAGES CXX)
  set(MR_PLANNER_LEGO_STANDALONE ON)
else()
  set(MR_PLANNER_LEGO_STANDALONE OFF)
endif()

option(MR_PLANNER_LEGO_ENABLE_VAMP "Enable VAMP via bundled mr_planner_core" ON)

set(MR_PLANNER_LEGO_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(MR_PLANNER_LEGO_MONOREPO_ROOT "${MR_PLANNER_LEGO_DIR}/..")

include(GNUInstallDirs)

if(MR_PLANNER_LEGO_ENABLE_VAMP)
  set(MR_PLANNER_CORE_ENABLE_VAMP_DEFAULT ON)
else()
  set(MR_PLANNER_CORE_ENABLE_VAMP_DEFAULT OFF)
endif()

set(MR_PLANNER_LEGO_CORE_TARGET "")

# Prefer an installed mr_planner_core (so mr_planner_lego can be its own repo), but
# fall back to the mono-repo subdirectory when available.
if(MR_PLANNER_LEGO_STANDALONE)
  if(EXISTS "${MR_PLANNER_LEGO_MONOREPO_ROOT}/mr_planner_core/CMakeLists.txt")
    option(MR_PLANNER_LEGO_USE_BUNDLED_CORE "Build mr_planner_core from the mono-repo checkout" ON)
  else()
    option(MR_PLANNER_LEGO_USE_BUNDLED_CORE "Build mr_planner_core from the mono-repo checkout" OFF)
  endif()

  if(MR_PLANNER_LEGO_USE_BUNDLED_CORE)
    set(MR_PLANNER_CORE_ENABLE_VAMP ${MR_PLANNER_CORE_ENABLE_VAMP_DEFAULT} CACHE BOOL "Build VAMP collision backend" FORCE)
    add_subdirectory("${MR_PLANNER_LEGO_MONOREPO_ROOT}/mr_planner_core" "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_core_build")
    set(MR_PLANNER_LEGO_CORE_TARGET mr_planner_core)
  else()
    find_package(mr_planner_core CONFIG REQUIRED)
    set(MR_PLANNER_LEGO_CORE_TARGET mr_planner::mr_planner_core)
  endif()
else()
  if(TARGET mr_planner_core)
    set(MR_PLANNER_LEGO_CORE_TARGET mr_planner_core)
  elseif(TARGET mr_planner::mr_planner_core)
    set(MR_PLANNER_LEGO_CORE_TARGET mr_planner::mr_planner_core)
  else()
    find_package(mr_planner_core CONFIG REQUIRED)
    set(MR_PLANNER_LEGO_CORE_TARGET mr_planner::mr_planner_core)
  endif()
endif()

add_library(mr_planner_lego SHARED
  src/version.cpp
  src/applications/lego/core/Lego.cpp
  src/applications/lego/lego_primitive.cpp
  src/applications/lego/lego_plan.cpp
)

target_compile_features(mr_planner_lego PUBLIC cxx_std_17)
target_compile_definitions(mr_planner_lego PUBLIC MR_PLANNER_WITH_ROS=0)

target_include_directories(mr_planner_lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mr_planner_lego PUBLIC ${MR_PLANNER_LEGO_CORE_TARGET})

if(MR_PLANNER_LEGO_STANDALONE)
  add_executable(mr_planner_lego_assign
    apps/mr_planner_lego_assign.cpp
  )
  target_link_libraries(mr_planner_lego_assign PRIVATE mr_planner_lego)

  add_executable(mr_planner_lego_plan
    apps/mr_planner_lego_plan.cpp
  )
  target_link_libraries(mr_planner_lego_plan PRIVATE mr_planner_lego)

  if(UNIX AND NOT APPLE)
    set_target_properties(mr_planner_lego_assign mr_planner_lego_plan PROPERTIES
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
    )
  endif()

  if(NOT MR_PLANNER_LEGO_USE_BUNDLED_CORE)
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
      "${MR_PLANNER_LEGO_DIR}/cmake/mr_planner_legoConfig.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_legoConfig.cmake"
      INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_lego"
    )
    write_basic_package_version_file(
      "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_legoConfigVersion.cmake"
      VERSION "${PROJECT_VERSION}"
      COMPATIBILITY SameMajorVersion
    )

    install(TARGETS mr_planner_lego
      EXPORT mr_planner_legoTargets
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )

    install(TARGETS mr_planner_lego_assign mr_planner_lego_plan
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )

    install(DIRECTORY include/
      DESTINATION include
    )

    install(EXPORT mr_planner_legoTargets
      NAMESPACE mr_planner::
      DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_lego"
    )

    install(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_legoConfig.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_legoConfigVersion.cmake"
      DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_lego"
    )
  endif()
endif()
