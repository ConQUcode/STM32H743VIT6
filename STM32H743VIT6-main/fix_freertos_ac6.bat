@echo off
setlocal

set SCRIPT_DIR=%~dp0
echo [%date% %time%] run fix_freertos_ac6.bat >> "%SCRIPT_DIR%fix_freertos_ac6.log"
powershell.exe -ExecutionPolicy Bypass -File "%SCRIPT_DIR%fix_freertos_ac6.ps1"
echo [%date% %time%] exit code %errorlevel% >> "%SCRIPT_DIR%fix_freertos_ac6.log"
exit /b %errorlevel%
