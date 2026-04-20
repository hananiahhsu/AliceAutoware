@echo off
setlocal EnableExtensions EnableDelayedExpansion

for %%I in ("%~dp0..") do set "ROOT_DIR=%%~fI"
if not exist "%ROOT_DIR%\CMakeLists.txt" set "ROOT_DIR=%~dp0"
set "OUT_DIR=%ROOT_DIR%\out"
set "LOG_DIR=%OUT_DIR%\logs"
if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"
set "LOG_FILE=%LOG_DIR%\diagnose_windows_env.log"

> "%LOG_FILE%" echo [MAD] Windows environment diagnosis
>> "%LOG_FILE%" echo DATE: %DATE% %TIME%
>> "%LOG_FILE%" echo ROOT_DIR: %ROOT_DIR%
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== where cmake ====
where cmake >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== cmake --version ====
cmake --version >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== where ninja ====
where ninja >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== ninja --version ====
ninja --version >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== where python ====
where python >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== python --version ====
python --version >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== py -3 -V ====
py -3 -V >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== where cl ====
where cl >> "%LOG_FILE%" 2>&1
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== VSWHERE ====
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
  "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 >> "%LOG_FILE%" 2>&1
) else (
  >> "%LOG_FILE%" echo vswhere.exe not found
)
>> "%LOG_FILE%" echo.
>> "%LOG_FILE%" echo ==== PATH ====
>> "%LOG_FILE%" echo %PATH%

echo [MAD] Diagnosis log written to:
echo [MAD]   %LOG_FILE%
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 0
