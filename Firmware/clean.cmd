@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0clean.ps1" %*
if errorlevel 1 pause