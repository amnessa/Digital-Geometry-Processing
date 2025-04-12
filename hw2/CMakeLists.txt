cmake_minimum_required(VERSION 3.10)
project(Assignment2)

# Use C++11 (or later)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define that we are using the DLL version of Coin3D and SoWin.
add_definitions(-DCOIN_DLL -DSOWIN_DLL)

# Set the path to the Coin3D include and lib directory.
include_directories("C:/Users/cagopa/Desktop/Digital-Geometry-Processing/Coin3D/include")
include_directories("C:/Users/cagopa/Desktop/Digital-Geometry-Processing/eigen-3.4.0")

# List your source files.
set(SOURCES
    "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/main.cpp"
    "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/Mesh.cpp"
    "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/Painter.cpp"
)

# Create the executable target.
add_executable(Assignment2 ${SOURCES})

# Specify the directory where the Coin3D libraries are located.
link_directories("C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2")

# Link against the required Coin3D libraries.
target_link_libraries(Assignment2 
    "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/coin2d.lib"
    "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/sowin1d.lib"
)

# Copy the required DLLs to the output directory after building.
add_custom_command(TARGET Assignment2 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/Coin3D/bin/coin2d.dll"
        $<TARGET_FILE_DIR:Assignment2>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/Coin3D/bin/sowin1d.dll"
        $<TARGET_FILE_DIR:Assignment2>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/msvcr71d.dll"
        $<TARGET_FILE_DIR:Assignment2>
)
