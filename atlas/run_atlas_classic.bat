@echo off
rem ATLAS with the classic Tkinter interface (no pip packages needed).
cd /d "%~dp0"
py -3 atlas_v9.py
if errorlevel 9009 python atlas_v9.py
pause
