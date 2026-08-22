@echo off
set ACLI="C:\Program Files\Arduino CLI\arduino-cli.exe"
set SKETCH="D:\1-EXTERNAL\PROYECTOS JMS 2025\SMART-TROLLEY-JUN-2026\MOTOR-INTERFACE-V14"
set PORT=COM4
set FQBN=arduino:avr:mega

echo [1/2] Compilando firmware...
%ACLI% compile --fqbn %FQBN% %SKETCH%
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Compilacion fallida
    pause
    exit /b 1
)

echo.
echo [2/2] Subiendo firmware a %PORT%...
%ACLI% upload --fqbn %FQBN% --port %PORT% %SKETCH%
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Upload fallido - verifica que COM4 no este ocupado
    pause
    exit /b 1
)

echo.
echo ===================================
echo  FIRMWARE CARGADO CORRECTAMENTE
echo  Puerto: %PORT%  Placa: Mega 2560
echo ===================================
pause
