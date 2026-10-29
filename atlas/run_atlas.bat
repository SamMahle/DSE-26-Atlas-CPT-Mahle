@echo off
cd /d "%~dp0"
py -3 atlas_v9.py
if errorlevel 1 python atlas_v9.py
pause
