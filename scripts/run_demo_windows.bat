@echo off
setlocal EnableExtensions EnableDelayedExpansion

for %%I in ("%~dp0..") do set "ROOT_DIR=%%~fI"
if not exist "%ROOT_DIR%\CMakeLists.txt" set "ROOT_DIR=%~dp0"
set "SCENARIO=%~1"
if "%SCENARIO%"=="" set "SCENARIO=highway_lane_change"

call :pick_bin mad_demo.exe BIN
if not defined BIN (
  echo [MAD] demo binary not found.
  echo [MAD] Build first with scripts\build_windows.bat
  echo [MAD] Searched under out\build and out\install.
  if /i not "%MAD_NO_PAUSE%"=="1" pause
  exit /b 2
)

echo [MAD] Running %SCENARIO%
echo [MAD] Using %BIN%
"%BIN%" "%SCENARIO%"
set "RET=%ERRORLEVEL%"
if not "%RET%"=="0" (
  echo [MAD] demo failed with exit code %RET%
  if /i not "%MAD_NO_PAUSE%"=="1" pause
  exit /b %RET%
)
if /i not "%MAD_NO_PAUSE%"=="1" pause
exit /b 0

:pick_bin
set "%~2="
for %%P in (
  "%ROOT_DIR%\out\build\windows-ninja\bin\%~1"
  "%ROOT_DIR%\out\build\windows-ninja\%~1"
  "%ROOT_DIR%\out\build\windows-vs2022\bin\Release\%~1"
  "%ROOT_DIR%\out\build\windows-vs2022\Release\%~1"
  "%ROOT_DIR%\out\build\windows-vs2019\bin\Release\%~1"
  "%ROOT_DIR%\out\build\windows-vs2019\Release\%~1"
  "%ROOT_DIR%\out\install\windows-ninja\bin\%~1"
  "%ROOT_DIR%\out\install\windows-vs2022\bin\%~1"
  "%ROOT_DIR%\out\install\windows-vs2019\bin\%~1"
) do (
  if exist %%~P (
    set "%~2=%%~P"
    exit /b 0
  )
)
exit /b 1
