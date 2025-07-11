cmake_minimum_required(VERSION 3.10)
project(PairwiseHarmonicsSegmentation)

# Use C++17 for modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define DLL usage for Coin3D and SoWin
add_definitions(-DCOIN_DLL -DSOWIN_DLL)

# Fix compatibility issues with MSVC
if(MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-DEIGEN_STRONG_INLINE=inline)
    # Disable certain warnings for cleaner output
    add_compile_options(/wd4996 /wd4267 /wd4244)
    # Fix object file size limit for large template instantiations
    add_compile_options(/bigobj)
endif()

# Find required dependencies
find_path(COIN3D_INCLUDE_DIR NAMES Inventor/Win/SoWin.h
          PATHS "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/Coin3D/include"
          NO_DEFAULT_PATH)
find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Dense
          PATHS "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/eigen-install/include/eigen3"
          NO_DEFAULT_PATH)
find_path(LIBIGL_INCLUDE_DIR NAMES igl/cotmatrix.h
          PATHS "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/libigl-2.6.0/include"
          NO_DEFAULT_PATH)

# Check and include dependencies
if(COIN3D_INCLUDE_DIR)
    include_directories(${COIN3D_INCLUDE_DIR})
    message(STATUS "Found Coin3D: ${COIN3D_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Coin3D not found!")
endif()

if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found!")
endif()

if(LIBIGL_INCLUDE_DIR)
    include_directories(${LIBIGL_INCLUDE_DIR})
    message(STATUS "Found libigl: ${LIBIGL_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "libigl not found!")
endif()

# Core source files for the clean implementation
set(CORE_SOURCES
    main_segmentation.cpp
    Mesh.cpp
    Painter.cpp
    helpers.cpp
    PairwiseHarmonicsSegmentation.cpp
    IsocurveAnalysis.cpp
    RigidityAnalysis.cpp
    VisualizationUtils.cpp
    AnimatedMeshPipeline.cpp
)

# Create the main executable
add_executable(PairwiseHarmonicsSegmentation ${CORE_SOURCES})

# Link directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# Link required libraries
target_link_libraries(PairwiseHarmonicsSegmentation
    ${CMAKE_SOURCE_DIR}/libs/coin2d.lib
    ${CMAKE_SOURCE_DIR}/libs/sowin1d.lib
)

# Post-build commands to copy required files
add_custom_command(TARGET PairwiseHarmonicsSegmentation POST_BUILD
    # Copy required DLLs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/dynamic_link_lib/coin2d.dll"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/dynamic_link_lib/sowin1d.dll"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/dynamic_link_lib/msvcr71d.dll"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>    # Copy mesh files to executable directory (not subdirectory)
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/50.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/249.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/348.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/man0.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/Armadillo.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/chair.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/cow.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/models/hand.off"
        $<TARGET_FILE_DIR:PairwiseHarmonicsSegmentation>/
)

# Set target properties
set_target_properties(PairwiseHarmonicsSegmentation PROPERTIES
    OUTPUT_NAME "PairwiseHarmonicsSegmentation"
    DEBUG_POSTFIX "_debug"
)

message(STATUS "Clean Pairwise Harmonics Segmentation configured successfully!")
