@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0upload.ps1" %*
if errorlevel 1 pause