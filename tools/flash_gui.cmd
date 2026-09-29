@echo off
REM One-click launcher for the RTKdata edge flasher GUI.
REM RTK_ENROLL_MASTER must be set as a User env var on this machine.
python "%~dp0flash_gui.py"
if errorlevel 1 pause
