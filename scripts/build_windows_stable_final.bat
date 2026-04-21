@echo off
setlocal EnableExtensions

for %%I in ("%~dp0..") do set "ROOT_DIR=%%~fI"
if not exist "%ROOT_DIR%\CMakeLists.txt" (
  set "ROOT_DIR=%~dp0"
  if not exist "%ROOT_DIR%\CMakeLists.txt" (
    echo [MAD] ERROR: Could not infer repository root from %~dp0
    if /i not "%MAD_NO_PAUSE%"=="1" pause
    exit /b 1
  )
)

set "OUT_DIR=%ROOT_DIR%\out"
set "LOG_DIR=%OUT_DIR%\logs"
if not exist "%LOG_DIR%" mkdir "%LOG_DIR%" >nul 2>nul

call :make_run_stamp
set "CONFIG_LOG=%LOG_DIR%\configure_%RUN_STAMP%.log"
set "BUILD_LOG=%LOG_DIR%\build_%RUN_STAMP%.log"
set "TEST_LOG=%LOG_DIR%\test_%RUN_STAMP%.log"
set "INSTALL_LOG=%LOG_DIR%\install_%RUN_STAMP%.log"
set "ARCH_LOG=%LOG_DIR%\architecture_%RUN_STAMP%.log"

set "BUILD_DIR="
set "INSTALL_DIR="
set "GENERATOR="
set "MULTI_CONFIG=0"
set "PYTHON_CMD=python"
set "PYTHON_ARGS="

call :banner "Root directory : %ROOT_DIR%"
call :banner "Output dir     : %OUT_DIR%"
call :banner "Configure log  : %CONFIG_LOG%"
call :banner "Build log      : %BUILD_LOG%"
call :banner "Test log       : %TEST_LOG%"
call :banner "Install log    : %INSTALL_LOG%"
call :banner "Arch log       : %ARCH_LOG%"

where cmake >nul 2>nul
if errorlevel 1 (
  call :fail "cmake was not found in PATH. Install CMake and try again."
  goto :done_fail
)

where python >nul 2>nul
if errorlevel 1 (
  py -3 -V >nul 2>nul
  if errorlevel 1 (
    call :fail "Neither python nor py -3 was found in PATH. Install Python 3 and try again."
    goto :done_fail
  )
  set "PYTHON_CMD=py"
  set "PYTHON_ARGS=-3"
)

where ninja >nul 2>nul
if not errorlevel 1 (
  call :ensure_ninja_compiler
  if not errorlevel 1 (
    set "GENERATOR=Ninja"
    set "BUILD_DIR=%OUT_DIR%\build\windows-ninja"
    set "INSTALL_DIR=%OUT_DIR%\install\windows-ninja"
    set "MULTI_CONFIG=0"
    call :banner "Using generator: Ninja"
  )
)

if not defined GENERATOR (
  call :setup_vs
  if errorlevel 1 (
    call :fail "Ninja not found and Visual Studio C++ tools were not detected."
    goto :done_fail
  )
)

if exist "%BUILD_DIR%\CMakeCache.txt" (
  call :banner "Cleaning stale CMake cache: %BUILD_DIR%\CMakeCache.txt"
  del /f /q "%BUILD_DIR%\CMakeCache.txt" >nul 2>nul
)
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%" >nul 2>nul
if not exist "%INSTALL_DIR%" mkdir "%INSTALL_DIR%" >nul 2>nul

if "%MULTI_CONFIG%"=="0" goto :do_ninja

call :banner "cmake -S \"%ROOT_DIR%\" -B \"%BUILD_DIR%\" -G \"%GENERATOR%\" -A x64 -DCMAKE_INSTALL_PREFIX:PATH=\"%INSTALL_DIR%\" -DMAD_BUILD_TESTS:BOOL=ON \"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE:PATH=%BUILD_DIR%\bin\Release\""
cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" -G "%GENERATOR%" -A x64 -DCMAKE_INSTALL_PREFIX:PATH="%INSTALL_DIR%" -DMAD_BUILD_TESTS:BOOL=ON "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE:PATH=%BUILD_DIR%\bin\Release" > "%CONFIG_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %CONFIG_LOG%"
  goto :done_fail
)

call :banner "cmake --build \"%BUILD_DIR%\" --config Release --parallel"
cmake --build "%BUILD_DIR%" --config Release --parallel > "%BUILD_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %BUILD_LOG%"
  goto :done_fail
)

call :banner "ctest --test-dir \"%BUILD_DIR%\" -C Release --output-on-failure"
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure > "%TEST_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %TEST_LOG%"
  goto :done_fail
)

call :banner "cmake --install \"%BUILD_DIR%\" --config Release"
cmake --install "%BUILD_DIR%" --config Release > "%INSTALL_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %INSTALL_LOG%"
  goto :done_fail
)

goto :do_arch

:do_ninja
call :banner "cmake -S \"%ROOT_DIR%\" -B \"%BUILD_DIR%\" -G \"Ninja\" -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INSTALL_PREFIX:PATH=\"%INSTALL_DIR%\" -DMAD_BUILD_TESTS:BOOL=ON \"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=%BUILD_DIR%\bin\""
cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" -G "Ninja" -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INSTALL_PREFIX:PATH="%INSTALL_DIR%" -DMAD_BUILD_TESTS:BOOL=ON "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=%BUILD_DIR%\bin" > "%CONFIG_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %CONFIG_LOG%"
  goto :done_fail
)

call :banner "cmake --build \"%BUILD_DIR%\" --parallel"
cmake --build "%BUILD_DIR%" --parallel > "%BUILD_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %BUILD_LOG%"
  goto :done_fail
)

call :banner "ctest --test-dir \"%BUILD_DIR%\" --output-on-failure"
ctest --test-dir "%BUILD_DIR%" --output-on-failure > "%TEST_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %TEST_LOG%"
  goto :done_fail
)

call :banner "cmake --install \"%BUILD_DIR%\""
cmake --install "%BUILD_DIR%" > "%INSTALL_LOG%" 2>&1
if errorlevel 1 (
  call :fail "Command failed. Review log: %INSTALL_LOG%"
  goto :done_fail
)

:do_arch
if "%PYTHON_CMD%"=="python" (
  call :banner "python \"%ROOT_DIR%\tools\check_architecture_sync.py\""
  python "%ROOT_DIR%\tools\check_architecture_sync.py" > "%ARCH_LOG%" 2>&1
) else (
  call :banner "py -3 \"%ROOT_DIR%\tools\check_architecture_sync.py\""
  py -3 "%ROOT_DIR%\tools\check_architecture_sync.py" > "%ARCH_LOG%" 2>&1
)
if errorlevel 1 (
  call :fail "Command failed. Review log: %ARCH_LOG%"
  goto :done_fail
)

call :success_summary
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 0

:ensure_ninja_compiler
where cl >nul 2>nul
if not errorlevel 1 exit /b 0
where g++ >nul 2>nul
if not errorlevel 1 exit /b 0
where clang++ >nul 2>nul
if not errorlevel 1 exit /b 0
call :setup_vs_env_only
if errorlevel 1 exit /b 1
where cl >nul 2>nul
if not errorlevel 1 exit /b 0
exit /b 1

:setup_vs_env_only
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
set "VS_PATH="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%I"
if not defined VS_PATH exit /b 1
if not exist "%VS_PATH%\Common7\Tools\VsDevCmd.bat" exit /b 1
call :banner "Configuring Visual Studio environment for Ninja: %VS_PATH%"
call "%VS_PATH%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul 2>nul
exit /b %ERRORLEVEL%

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
call "%VS_PATH%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul 2>nul
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
call :banner "Using generator: %GENERATOR%"
exit /b 0

:success_summary
call :banner "[MAD] Build completed successfully."
call :banner "Build dir      : %BUILD_DIR%"
call :banner "Install dir    : %INSTALL_DIR%"
call :banner "Expected binaries:"
call :banner "  %BUILD_DIR%\bin\mad_demo.exe"
call :banner "  %BUILD_DIR%\bin\mad_batch_eval.exe"
call :banner "  %BUILD_DIR%\bin\mad_visualizer.exe"
call :banner "  %BUILD_DIR%\bin\mad_tests.exe"
if "%MULTI_CONFIG%"=="1" (
  call :banner "  %BUILD_DIR%\bin\Release\mad_demo.exe"
  call :banner "  %BUILD_DIR%\bin\Release\mad_batch_eval.exe"
  call :banner "  %BUILD_DIR%\bin\Release\mad_visualizer.exe"
  call :banner "  %BUILD_DIR%\bin\Release\mad_tests.exe"
)
call :banner "Installed binaries:"
call :banner "  %INSTALL_DIR%\bin\mad_demo.exe"
call :banner "  %INSTALL_DIR%\bin\mad_batch_eval.exe"
call :banner "  %INSTALL_DIR%\bin\mad_visualizer.exe"
call :banner "Configure log  : %CONFIG_LOG%"
call :banner "Build log      : %BUILD_LOG%"
call :banner "Test log       : %TEST_LOG%"
call :banner "Install log    : %INSTALL_LOG%"
call :banner "Arch log       : %ARCH_LOG%"
exit /b 0

:banner
echo %~1
exit /b 0

:fail
call :banner "[MAD] ERROR: %~1"
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 1

:make_run_stamp
set "RUN_STAMP="
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "(Get-Date).ToString('yyyyMMdd_HHmmss_fff')" 2^>nul`) do set "RUN_STAMP=%%I"
if not defined RUN_STAMP set "RUN_STAMP=%RANDOM%_%RANDOM%"
exit /b 0

:done_fail
exit /b 1
