cmake_minimum_required(VERSION 3.10)
project(Term_Project)

# Use C++11 (or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define that we are using the DLL version of Coin3D and SoWin.
add_definitions(-DCOIN_DLL -DSOWIN_DLL)

# Add preprocessor definitions to fix Eigen compatibility issues with VS2022
if(MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-DEIGEN_STRONG_INLINE=inline)
endif()

# Set the path to the Coin3D include and Eigen include directories.
# Using environment variables or find_package would be better for portability
find_path(COIN3D_INCLUDE_DIR NAMES Inventor/Win/SoWin.h
          PATHS "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/Coin3D/include"
          NO_DEFAULT_PATH)
find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Dense
          PATHS "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/eigen-install/include/eigen3"
          NO_DEFAULT_PATH)

if(COIN3D_INCLUDE_DIR)
    include_directories(${COIN3D_INCLUDE_DIR})
    message(STATUS "Found Coin3D include directory: ${COIN3D_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Could not find Coin3D include directory")
endif()

if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "Found Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Could not find Eigen3 include directory")
endif()

# List your source files.
set(SOURCES
    main_segmentation.cpp
    Mesh.cpp
    Painter.cpp
    helpers.cpp
)

# Create the executable target.
add_executable(Term_Project ${SOURCES})

# Specify the directory where the Coin3D libraries are located.
link_directories(${CMAKE_SOURCE_DIR})

# Link against the required Coin3D libraries.
target_link_libraries(Term_Project
    ${CMAKE_SOURCE_DIR}/coin2d.lib
    ${CMAKE_SOURCE_DIR}/sowin1d.lib
)

# Copy the required DLLs and mesh files to the output directory after building.
add_custom_command(TARGET Term_Project POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/coin2d.dll"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/sowin1d.dll"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/msvcr71d.dll"
        $<TARGET_FILE_DIR:Term_Project>
    # Copy mesh files
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/50.off"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/249.off"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/348.off"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/man0.off"
        $<TARGET_FILE_DIR:Term_Project>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/Armadillo.off"
        $<TARGET_FILE_DIR:Term_Project>
)
