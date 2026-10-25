@echo off
rem Copyright (c) 2026 Ohm Lab Ltd.
rem SPDX-License-Identifier: MIT
setlocal enableextensions
rem Finds a Python interpreter and runs it with %*.
rem NEURON6_PYTHON overrides the search.

set "PYEXE="
set "PYARGS="

rem 0) NEURON6_PYTHON
if defined NEURON6_PYTHON if exist "%NEURON6_PYTHON%" set "PYEXE=%NEURON6_PYTHON%"

rem 1) the py launcher
if not defined PYEXE (
  where py >nul 2>nul
  if not errorlevel 1 (
    set "PYEXE=py"
    set "PYARGS=-3"
  )
)

rem 2) python.exe on PATH, skipping the WindowsApps alias. findstr is called
rem    by its full path so that another findstr on PATH is not run.
if not defined PYEXE (
  for /f "delims=" %%P in ('where python 2^>nul') do (
    if not defined PYEXE (
      echo %%P| "%SystemRoot%\System32\findstr.exe" /i /c:"WindowsApps" >nul
      if errorlevel 1 set "PYEXE=%%P"
    )
  )
)

rem 3) conda and per-user installs
if not defined PYEXE (
  for %%P in (
    "%USERPROFILE%\miniconda3\python.exe"
    "%USERPROFILE%\anaconda3\python.exe"
    "%ProgramData%\miniconda3\python.exe"
    "%ProgramData%\anaconda3\python.exe"
  ) do (
    if not defined PYEXE if exist "%%~P" set "PYEXE=%%~P"
  )
)

if not defined PYEXE (
  echo ERROR: run_python.cmd could not find a real Python interpreter. 1>&2
  echo        Install Python ^(python.org or conda^) or set NEURON6_PYTHON. 1>&2
  exit /b 9009
)

rem Outside any block, so errorlevel is Python's exit code.
"%PYEXE%" %PYARGS% %*
exit /b %errorlevel%
