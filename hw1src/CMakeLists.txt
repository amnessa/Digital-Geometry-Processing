cmake_minimum_required(VERSION 3.10.0)
project(hw1 VERSION 0.1.0 LANGUAGES C CXX)

# Set the Coin3D include and library directories.
# These can be overridden via the environment variables COIN3D_INCLUDE_DIR and COIN3D_LIB_DIR.
if(DEFINED ENV{COIN3D_INCLUDE_DIR})
    set(COIN3D_INCLUDE_DIR $ENV{COIN3D_INCLUDE_DIR})
else()
    set(COIN3D_INCLUDE_DIR "/home/cago/Digital-Geometry-Processing/Coin3D/include")
endif()

if(DEFINED ENV{COIN3D_LIB_DIR})
    set(COIN3D_LIB_DIR $ENV{COIN3D_LIB_DIR})
else()
    set(COIN3D_LIB_DIR "/home/cago/Digital-Geometry-Processing/Coin3D/lib")
endif()

# Add the Coin3D include directory.
include_directories(${COIN3D_INCLUDE_DIR})

# Add the Coin3D library directory.
link_directories(${COIN3D_LIB_DIR})

# Add the executable source.
add_executable(hw1 main.cpp)

# (Optional) If required by Coin3D, define preprocessor symbols.
target_compile_definitions(hw1 PRIVATE COIN_DLL SOWIN_DLL)

# Link against the Coin3D libraries.
# On Linux the libraries are typically named libcoin2d.so and libsowin1d.so.
target_link_libraries(hw1 coin2d sowin1d)

