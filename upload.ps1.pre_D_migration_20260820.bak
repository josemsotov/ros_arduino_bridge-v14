$env:PATH = [System.Environment]::GetEnvironmentVariable("PATH","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("PATH","User")
$acli   = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$sketch = "F:\PROYECTOS JMS 2025\SMART-TROLLEY-JUN-2026\MOTOR-INTERFACE-V14"
$fqbn   = "arduino:avr:mega"
$port   = "COM4"

Write-Host "[1/2] Compilando firmware..." -ForegroundColor Cyan
& $acli compile --fqbn $fqbn $sketch
if ($LASTEXITCODE -ne 0) { Write-Host "ERROR en compilacion" -ForegroundColor Red; exit 1 }

Write-Host ""
Write-Host "[2/2] Subiendo firmware a $port..." -ForegroundColor Cyan
& $acli upload --fqbn $fqbn --port $port $sketch
if ($LASTEXITCODE -ne 0) { Write-Host "ERROR en upload" -ForegroundColor Red; exit 1 }

Write-Host ""
Write-Host "=====================================" -ForegroundColor Green
Write-Host " FIRMWARE CARGADO CORRECTAMENTE     " -ForegroundColor Green
Write-Host " Puerto: $port  |  Placa: Mega 2560  " -ForegroundColor Green
Write-Host "=====================================" -ForegroundColor Green
