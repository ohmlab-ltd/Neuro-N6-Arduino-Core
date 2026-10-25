@echo off
REM Copyright (c) 2026 Ohm Lab Ltd.
REM SPDX-License-Identifier: MIT
REM Builds the uploader GUI as dist\Neuro N6 DFU.exe.
REM Needs: pip install pyinstaller pyserial
cd /d "%~dp0"

pyinstaller --noconfirm --onefile --windowed ^
  --name "Neuro N6 DFU" ^
  --icon "SoM-3.ico" ^
  --add-data "SoM-3.ico;." ^
  neuro-n6-uploader.py

echo.
if exist "dist\Neuro N6 DFU.exe" (
  echo Built: dist\Neuro N6 DFU.exe
) else (
  echo BUILD FAILED - see output above
)
pause
