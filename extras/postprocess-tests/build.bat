@echo off
rem Copyright (c) 2026 Ohm Lab Ltd.
rem SPDX-License-Identifier: MIT
rem Builds test_zoo.exe from the real post-processor sources plus ST's
rem decoders, with the headers in shim\ standing in for the board's runtime.
rem Everything it writes goes to %N6_PP_WORK% (default %TEMP%\n6-postprocess-tests),
rem so nothing generated lands in the repository. Needs the MSVC build tools;
rem see README.md.
setlocal
if not defined VC set "VC=%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VC%" set "VC=%ProgramFiles%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VC%" set "VC=%ProgramFiles(x86)%\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VC%" (
  echo Could not find vcvars64.bat; set VC to it and run again.
  exit /b 1
)
call "%VC%" >nul || exit /b 1

set HERE=%~dp0
set ROOT=%HERE%..\..
if "%N6_PP_WORK%"=="" set N6_PP_WORK=%TEMP%\n6-postprocess-tests
set CORE=%ROOT%\cores\NeuroN6
set PP=%ROOT%\libraries\PostProcess\src
set VMP=%CORE%\lib_vision_models_pp\lib_vision_models_pp

if not exist "%N6_PP_WORK%\obj" mkdir "%N6_PP_WORK%\obj"

cl /nologo /W3 /std:c17 /O2 /MD /wd4996 ^
  /FI"%HERE%shim\host_prefix.h" ^
  /I"%HERE%shim" /I"%PP%" /I"%VMP%\Inc" /I"%VMP%\Src" /I"%CORE%" ^
  /Fd"%N6_PP_WORK%\obj\test_zoo.pdb" ^
  /Fo"%N6_PP_WORK%\obj\\" /Fe"%N6_PP_WORK%\test_zoo.exe" ^
  "%HERE%test_zoo.c" ^
  "%PP%\zoo_pp.c" "%PP%\zoo_pp_lcd.c" ^
  "%PP%\PostProcessor_ZooDetect.c" "%PP%\PostProcessor_ZooPose.c" ^
  "%PP%\PostProcessor_ZooSeg.c" "%PP%\PostProcessor_Classifier.c" ^
  "%PP%\vnd_meta.c" "%PP%\ohmlab_lcd_draw.c" "%PP%\Fonts\font16.c" "%PP%\yolox_pp.c" ^
  "%VMP%\Src\od_pp_yolov8.c" "%VMP%\Src\od_pp_st_yolox.c" "%VMP%\Src\od_pp_yolov2.c" ^
  "%VMP%\Src\od_pp_fd_blazeface.c" "%VMP%\Src\spe_movenet_pp.c" ^
  "%VMP%\Src\vision_models_pp.c" "%VMP%\Src\vision_models_pp_maxi_is8.c" ^
  "%VMP%\Src\vision_models_pp_maxi_iu8.c" "%VMP%\Src\vision_models_pp_maxi_if32.c"
exit /b %errorlevel%
