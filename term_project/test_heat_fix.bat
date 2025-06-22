@echo off
echo Testing Heat Geodesics Fix
echo =========================
echo.
echo This script will test if the LibIGL conversion ordering fix resolves the heat geodesics issue.
echo.
echo Key fixes applied:
echo 1. Moved g_harmonics creation AFTER g_mesh->convertToLibIGL()
echo 2. Updated PairwiseHarmonics::initializeLibIGL() to use mesh's existing LibIGL data
echo 3. Fixed color coding: green=rigid, red=non-rigid
echo.
echo Build and test the application...
echo.
cd build
cmake --build . --config Debug
if %errorlevel% equ 0 (
    echo Build successful!
    echo Please run the application and test option 13 (Heat Geodesics)
    echo The distances should no longer be all zeros.
) else (
    echo Build failed! Check for compilation errors.
)
pause
