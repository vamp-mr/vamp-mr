cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(mr_planner_core VERSION 0.1.0 LANGUAGES CXX)
  set(MR_PLANNER_CORE_STANDALONE ON)
else()
  set(MR_PLANNER_CORE_STANDALONE OFF)
endif()

option(MR_PLANNER_CORE_ENABLE_VAMP "Build VAMP collision backend" ON)
option(MR_PLANNER_CORE_ENABLE_PYTHON "Build Python bindings (pybind11)" ON)

set(MR_PLANNER_CORE_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include(GNUInstallDirs)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread serialization)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

set(MR_PLANNER_TBB_LINK "")
find_package(TBB QUIET)
if(TBB_FOUND)
  if(TARGET TBB::tbb)
    set(MR_PLANNER_TBB_LINK TBB::tbb)
  elseif(DEFINED TBB_LIBRARIES)
    set(MR_PLANNER_TBB_LINK ${TBB_LIBRARIES})
  endif()
endif()
if(NOT MR_PLANNER_TBB_LINK)
  find_library(MR_PLANNER_TBB_LINK NAMES tbb)
endif()

set(mr_planner_proto_files
  "${MR_PLANNER_CORE_ROOT}/proto/mr_planner_graph.proto"
)
protobuf_generate_cpp(MR_PLANNER_PROTO_SRCS MR_PLANNER_PROTO_HDRS ${mr_planner_proto_files})

set(mr_planner_core_sources
  src/version.cpp
  src/core/instance.cpp
  src/core/logger.cpp
  src/core/metrics.cpp
  src/core/task.cpp
  src/io/skillplan.cpp
  src/io/graph_proto.cpp
  src/planning/SingleAgentPlanner.cpp
  src/planning/composite_rrt.cpp
  src/planning/planner.cpp
  src/planning/prm.cpp
  src/planning/resampler.cpp
  src/planning/roadmap.cpp
  src/planning/rrt.cpp
  src/planning/rrt_connect.cpp
  src/planning/shortcutter.cpp
  src/planning/shortcutter_mt.cpp
  src/planning/sipp_rrt.cpp
  src/execution/tpg.cpp
  src/execution/adg.cpp
  src/execution/policy.cpp
  ${MR_PLANNER_PROTO_SRCS}
)

if(MR_PLANNER_CORE_ENABLE_VAMP)
  find_package(vamp CONFIG REQUIRED)
  list(APPEND mr_planner_core_sources
    src/backends/vamp_instance.cpp
    src/backends/vamp_env_factory.cpp
    src/backends/vamp_plugin_loader.cpp
  )
endif()

add_library(mr_planner_core SHARED ${mr_planner_core_sources})

target_compile_features(mr_planner_core PUBLIC cxx_std_17)
target_compile_definitions(mr_planner_core PUBLIC
  MR_PLANNER_WITH_ROS=0
)
if(MR_PLANNER_CORE_ENABLE_VAMP)
  target_compile_definitions(mr_planner_core PUBLIC MR_PLANNER_WITH_VAMP=1)
else()
  target_compile_definitions(mr_planner_core PUBLIC MR_PLANNER_WITH_VAMP=0)
endif()
target_compile_definitions(mr_planner_core PRIVATE _OS_UNIX)

target_include_directories(mr_planner_core SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

target_include_directories(mr_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mr_planner_core PUBLIC
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
  Boost::date_time
  Boost::thread
  Boost::serialization
  ${OMPL_LIBRARIES}
  yaml-cpp
  jsoncpp_lib
  ${Protobuf_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(mr_planner_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(MR_PLANNER_TBB_LINK)
  target_link_libraries(mr_planner_core PUBLIC ${MR_PLANNER_TBB_LINK})
endif()

if(MR_PLANNER_CORE_ENABLE_VAMP)
  target_link_libraries(mr_planner_core PUBLIC vamp::vamp)
  if(CMAKE_DL_LIBS)
    target_link_libraries(mr_planner_core PRIVATE ${CMAKE_DL_LIBS})
  endif()
endif()

target_compile_options(mr_planner_core PUBLIC
  "-include"
  "mr_planner/common/eigen_config.h"
)

if(MR_PLANNER_CORE_STANDALONE)
  if(MR_PLANNER_CORE_ENABLE_VAMP)
    add_executable(mr_planner_core_plan
      apps/mr_planner_core_plan.cpp
    )
    target_link_libraries(mr_planner_core_plan PRIVATE mr_planner_core)
    if(UNIX AND NOT APPLE)
      set_target_properties(mr_planner_core_plan PROPERTIES
        INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
      )
    endif()

    add_executable(mr_planner_core_skillplan_to_tpg
      apps/mr_planner_core_skillplan_to_tpg.cpp
    )
    target_link_libraries(mr_planner_core_skillplan_to_tpg PRIVATE mr_planner_core)
    if(UNIX AND NOT APPLE)
      set_target_properties(mr_planner_core_skillplan_to_tpg PROPERTIES
        INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
      )
    endif()
  else()
    message(STATUS "MR_PLANNER_CORE_ENABLE_VAMP=OFF: skipping mr_planner_core_plan (requires VAMP).")
  endif()

  if(MR_PLANNER_CORE_ENABLE_PYTHON)
    if(NOT MR_PLANNER_CORE_ENABLE_VAMP)
      message(STATUS "MR_PLANNER_CORE_ENABLE_PYTHON=ON but VAMP is disabled; skipping Python bindings.")
    else()
      include(FetchContent)
      FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.12.0
      )
      FetchContent_MakeAvailable(pybind11)

      set(_mr_planner_core_python_executable "")
      if(DEFINED PYTHON_EXECUTABLE)
        set(_mr_planner_core_python_executable "${PYTHON_EXECUTABLE}")
      elseif(DEFINED Python3_EXECUTABLE)
        set(_mr_planner_core_python_executable "${Python3_EXECUTABLE}")
      endif()
      if(NOT _mr_planner_core_python_executable)
        message(FATAL_ERROR "Python interpreter not found (PYTHON_EXECUTABLE is missing).")
      endif()

      execute_process(
        COMMAND "${_mr_planner_core_python_executable}" -c
                "import os, site, sysconfig; prefix=r'${CMAKE_INSTALL_PREFIX}'.rstrip(os.sep); c=[p for p in site.getsitepackages() if p.startswith(prefix + os.sep)]; p=max(c, key=len) if c else sysconfig.get_path('platlib', vars={'base': prefix, 'platbase': prefix}); print(p)"
        OUTPUT_VARIABLE _mr_planner_core_py_platlib
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
      file(RELATIVE_PATH _mr_planner_core_py_install_dir "${CMAKE_INSTALL_PREFIX}" "${_mr_planner_core_py_platlib}")
      if(NOT DEFINED MR_PLANNER_CORE_PYTHON_INSTALL_DIR)
        set(MR_PLANNER_CORE_PYTHON_INSTALL_DIR "${_mr_planner_core_py_install_dir}"
            CACHE PATH "Python install directory (relative to CMAKE_INSTALL_PREFIX)")
      endif()

      set(_mr_planner_core_py_pkg_dir "${CMAKE_CURRENT_BINARY_DIR}/python/mr_planner_core")
      file(MAKE_DIRECTORY "${_mr_planner_core_py_pkg_dir}")

      set(_mr_planner_core_python_files
        "${MR_PLANNER_CORE_ROOT}/python/mr_planner_core/__init__.py"
        "${MR_PLANNER_CORE_ROOT}/python/mr_planner_core/srdf.py"
      )

      pybind11_add_module(_mr_planner_core MODULE
        python/mr_planner_core_pybind.cpp
      )
      target_link_libraries(_mr_planner_core PRIVATE mr_planner_core)
      target_compile_features(_mr_planner_core PRIVATE cxx_std_17)

      set_target_properties(_mr_planner_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${_mr_planner_core_py_pkg_dir}"
      )
      if(UNIX AND NOT APPLE)
        set_target_properties(_mr_planner_core PROPERTIES
          INSTALL_RPATH "$ORIGIN/../../.."
        )
      endif()

      foreach(_py_file IN LISTS _mr_planner_core_python_files)
        get_filename_component(_py_name "${_py_file}" NAME)
        add_custom_command(
          TARGET _mr_planner_core
          POST_BUILD
          COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                  "${_py_file}"
                  "${_mr_planner_core_py_pkg_dir}/${_py_name}"
        )
      endforeach()

      install(TARGETS _mr_planner_core
        LIBRARY DESTINATION "${MR_PLANNER_CORE_PYTHON_INSTALL_DIR}/mr_planner_core"
      )
      install(FILES ${_mr_planner_core_python_files}
        DESTINATION "${MR_PLANNER_CORE_PYTHON_INSTALL_DIR}/mr_planner_core"
      )
    endif()
  endif()

  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${MR_PLANNER_CORE_ROOT}/cmake/mr_planner_coreConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_coreConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_core"
  )
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_coreConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
  )

  install(TARGETS mr_planner_core
    EXPORT mr_planner_coreTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  )

  if(TARGET mr_planner_core_plan)
    install(TARGETS mr_planner_core_plan
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
  endif()

  if(TARGET mr_planner_core_skillplan_to_tpg)
    install(TARGETS mr_planner_core_skillplan_to_tpg
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  install(FILES "${MR_PLANNER_CORE_ROOT}/proto/mr_planner_graph.proto"
    DESTINATION share/mr_planner_core/proto
  )

  install(DIRECTORY "${MR_PLANNER_CORE_ROOT}/config/srdf"
    DESTINATION share/mr_planner_core/config
    FILES_MATCHING PATTERN "*.srdf"
  )

  install(EXPORT mr_planner_coreTargets
    NAMESPACE mr_planner::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_core"
  )

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_coreConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mr_planner_coreConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mr_planner_core"
  )
endif()
