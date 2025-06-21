@echo off
echo === Pairwise Harmonics Segmentation - Clean Build ===
echo.

REM Check if build directory exists
if not exist "build_clean" (
    echo Creating clean build directory...
    mkdir build_clean
)

cd build_clean

echo Configuring with CMake...
cmake .. -DCMAKE_BUILD_TYPE=Debug

if %ERRORLEVEL% neq 0 (
    echo Error: CMake configuration failed!
    pause
    exit /b 1
)

echo Building project...
cmake --build . --config Debug

if %ERRORLEVEL% neq 0 (
    echo Error: Build failed!
    pause
    exit /b 1
)

echo.
echo === Build completed successfully! ===
echo.
echo To run the program:
echo   cd Debug
echo   PairwiseHarmonicsSegmentation.exe
echo.
echo Available test meshes:
echo   - man0.off
echo   - Armadillo.off
echo   - 50.off
echo   - 249.off
echo   - 348.off
echo.
pause
