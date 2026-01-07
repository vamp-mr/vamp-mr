set(_vamp_cpm_cached "${CMAKE_CURRENT_LIST_DIR}/CPM.cmake")
set(_vamp_cpm_binary "${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake")

if(EXISTS "${_vamp_cpm_cached}")
  set(_vamp_cpm_path "${_vamp_cpm_cached}")
else()
  if(NOT EXISTS "${_vamp_cpm_binary}")
    file(
      DOWNLOAD
      https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.1/CPM.cmake
      ${_vamp_cpm_binary}
      EXPECTED_HASH SHA256=117cbf2711572f113bab262933eb5187b08cfc06dce0714a1ee94f2183ddc3ec
    )
  endif()
  set(_vamp_cpm_path "${_vamp_cpm_binary}")
endif()

set(CPM_USE_LOCAL_PACKAGES ON)
include("${_vamp_cpm_path}")

unset(_vamp_cpm_path)
unset(_vamp_cpm_binary)
unset(_vamp_cpm_cached)
