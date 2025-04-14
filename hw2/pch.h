#pragma once

// This precompiled header defines macros and includes headers in the correct order
// to prevent conflicts between Windows macros and C++ libraries

// Prevent Windows macro definitions that conflict with standard C++ and Eigen
#define NOMINMAX 
#define WIN32_LEAN_AND_MEAN

// Undefine any problematic Windows macros that might already be defined
#ifdef near
#undef near
#endif

#ifdef far
#undef far
#endif

#ifdef GetObject
#undef GetObject
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

// Include standard headers first
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cfloat>
#include <cmath>
#include <queue>

// Include Eigen headers before Windows.h
// This ensures Eigen's templates don't get messed up by Windows macros
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Now it's safe to include Windows headers
#include <windows.h>