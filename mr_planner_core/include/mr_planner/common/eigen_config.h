// Project-wide Eigen alignment configuration. Include before any Eigen header
// so every translation unit (core library, plugins, bindings) compiles Eigen
// with the same alignment settings and stays ABI-compatible.
#pragma once
#define EIGEN_MAX_ALIGN_BYTES 16
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0