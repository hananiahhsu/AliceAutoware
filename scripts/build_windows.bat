@echo off
setlocal EnableExtensions EnableDelayedExpansion

for %%I in ("%~dp0..") do set "ROOT_DIR=%%~fI"
if not exist "%ROOT_DIR%\CMakeLists.txt" (
  rem fallback: if script is run standalone from outside repo/scripts, assume sibling repo root cannot be inferred
  set "ROOT_DIR=%~dp0"
  if exist "%ROOT_DIR%\CMakeLists.txt" (
    rem script copied into repo root
  ) else (
    echo [MAD] ERROR: Could not infer repository root from %~dp0
    echo [MAD] Put this script under repo\scripts or repo root.
    pause
    exit /b 1
  )
)

set "OUT_DIR=%ROOT_DIR%\out"
set "LOG_DIR=%OUT_DIR%\logs"
if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"
set "LOG_FILE=%LOG_DIR%\build_windows.log"
set "BUILD_DIR="
set "INSTALL_DIR="
set "GENERATOR="
set "MULTI_CONFIG=0"
set "PYTHON_CMD="
set "RUNTIME_OUTPUT_ARG="
set "RUNTIME_OUTPUT_ARG_REL="

> "%LOG_FILE%" echo [MAD] Windows build log
call :banner "Root directory : %ROOT_DIR%"
call :banner "Output dir     : %OUT_DIR%"
call :banner "Log file       : %LOG_FILE%"

where cmake >nul 2>nul
if errorlevel 1 (
  call :fail "cmake was not found in PATH. Install CMake and try again."
  goto :done_fail
)

where python >nul 2>nul
if not errorlevel 1 (
  set "PYTHON_CMD=python"
) else (
  py -3 -V >nul 2>nul
  if not errorlevel 1 (
    set "PYTHON_CMD=py -3"
  ) else (
    call :fail "Neither python nor py -3 was found in PATH. Install Python 3 and try again."
    goto :done_fail
  )
)

where ninja >nul 2>nul
if not errorlevel 1 (
  set "GENERATOR=Ninja"
  set "BUILD_DIR=%OUT_DIR%\build\windows-ninja"
  set "INSTALL_DIR=%OUT_DIR%\install\windows-ninja"
  set "MULTI_CONFIG=0"
  set "RUNTIME_OUTPUT_ARG=-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=%BUILD_DIR%\bin"
  call :banner "Using generator: Ninja"
) else (
  call :setup_vs
  if errorlevel 1 (
    call :fail "Ninja not found and Visual Studio C++ tools were not detected."
    goto :done_fail
  )
)

if exist "%BUILD_DIR%\CMakeCache.txt" (
  call :banner "Cleaning stale CMake cache: %BUILD_DIR%\CMakeCache.txt"
  del /f /q "%BUILD_DIR%\CMakeCache.txt" >> "%LOG_FILE%" 2>&1
)
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
if not exist "%INSTALL_DIR%" mkdir "%INSTALL_DIR%"

if "%MULTI_CONFIG%"=="0" (
  call :run cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%" -DMAD_BUILD_TESTS=ON %RUNTIME_OUTPUT_ARG%
  if errorlevel 1 goto :done_fail
  call :run cmake --build "%BUILD_DIR%" --parallel
  if errorlevel 1 goto :done_fail
  call :run ctest --test-dir "%BUILD_DIR%" --output-on-failure
  if errorlevel 1 goto :done_fail
  call :run cmake --install "%BUILD_DIR%"
  if errorlevel 1 goto :done_fail
) else (
  call :run cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" -G "%GENERATOR%" -A x64 -DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%" -DMAD_BUILD_TESTS=ON %RUNTIME_OUTPUT_ARG_REL%
  if errorlevel 1 goto :done_fail
  call :run cmake --build "%BUILD_DIR%" --config Release --parallel
  if errorlevel 1 goto :done_fail
  call :run ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
  if errorlevel 1 goto :done_fail
  call :run cmake --install "%BUILD_DIR%" --config Release
  if errorlevel 1 goto :done_fail
)

call :run cmd /c %PYTHON_CMD% "%ROOT_DIR%\tools\check_architecture_sync.py"
if errorlevel 1 goto :done_fail

call :success_summary
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 0

:setup_vs
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
set "VS_PATH="
set "VS_VERSION="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%I"
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion`) do set "VS_VERSION=%%I"
if not defined VS_PATH exit /b 1
if not exist "%VS_PATH%\Common7\Tools\VsDevCmd.bat" exit /b 1
call :banner "Configuring Visual Studio environment: %VS_PATH%"
call "%VS_PATH%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >> "%LOG_FILE%" 2>&1
for /f "tokens=1 delims=." %%I in ("%VS_VERSION%") do set "VS_MAJOR=%%I"
if "%VS_MAJOR%"=="17" (
  set "GENERATOR=Visual Studio 17 2022"
  set "BUILD_DIR=%OUT_DIR%\build\windows-vs2022"
  set "INSTALL_DIR=%OUT_DIR%\install\windows-vs2022"
) else if "%VS_MAJOR%"=="16" (
  set "GENERATOR=Visual Studio 16 2019"
  set "BUILD_DIR=%OUT_DIR%\build\windows-vs2019"
  set "INSTALL_DIR=%OUT_DIR%\install\windows-vs2019"
) else (
  exit /b 1
)
set "MULTI_CONFIG=1"
set "RUNTIME_OUTPUT_ARG_REL=-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=%BUILD_DIR%\bin\Release"
call :banner "Using generator: %GENERATOR%"
exit /b 0

:run
call :banner "%*"
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo [CMD] %*
call %* >> "%LOG_FILE%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %LOG_FILE%"
  exit /b 1
)
exit /b 0

:success_summary
call :banner "[MAD] Build completed successfully."
call :banner "Build dir   : %BUILD_DIR%"
call :banner "Install dir : %INSTALL_DIR%"
call :banner "Expected binaries:"
call :banner "  %BUILD_DIR%\bin\mad_demo.exe"
call :banner "  %BUILD_DIR%\bin\mad_batch_eval.exe"
call :banner "  %BUILD_DIR%\bin\mad_tests.exe"
if "%MULTI_CONFIG%"=="1" (
  call :banner "  %BUILD_DIR%\bin\Release\mad_demo.exe"
  call :banner "  %BUILD_DIR%\bin\Release\mad_batch_eval.exe"
  call :banner "  %BUILD_DIR%\bin\Release\mad_tests.exe"
)
call :banner "Installed binaries:"
call :banner "  %INSTALL_DIR%\bin\mad_demo.exe"
call :banner "  %INSTALL_DIR%\bin\mad_batch_eval.exe"
call :banner "Log file: %LOG_FILE%"
exit /b 0

:banner
echo %~1
>> "%LOG_FILE%" echo %~1
exit /b 0

:fail
echo [MAD] ERROR: %~1
>> "%LOG_FILE%" echo [MAD] ERROR: %~1
>> "%LOG_FILE%" echo [MAD] Build failed.
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 1

:done_fail
exit /b 1
