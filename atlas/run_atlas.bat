@echo off
rem ATLAS with the NiceGUI interface. Opens in your web browser.
rem Keep this window open while you use ATLAS; closing it stops ATLAS.
cd /d "%~dp0"
py -3 atlas_gui.py %*
if errorlevel 9009 python atlas_gui.py %*
pause
