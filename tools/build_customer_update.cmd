@echo off
REM Build the single-file customer updater .exe.
REM
REM Produces dist\RTKdata-Update-<ver>.exe with the two flash images baked in,
REM so the customer downloads ONE file, plugs the base station in and clicks.
REM No Python, no esptool install, no command line on their side.
REM
REM Usage:  build_customer_update.cmd 1.1.8
REM Prereq: pip install pyinstaller esptool pyserial

setlocal
if "%~1"=="" (
  echo Usage: build_customer_update.cmd ^<version^>   e.g. 1.1.8
  exit /b 1
)
set VER=%~1
set ROOT=%~dp0..
set SHIP=%ROOT%\dist-ship-%VER%

if not exist "%SHIP%\rtkdata-fw.bin" (
  echo Missing %SHIP%\rtkdata-fw.bin
  echo Create the dist-ship-%VER% folder from a build first.
  exit /b 1
)

REM --add-data puts the images INSIDE the exe; customer_update.py reads them
REM from sys._MEIPASS when frozen. esptool is imported as a module at runtime,
REM so it must be collected explicitly.
pyinstaller --noconfirm --onefile --windowed ^
  --name "RTKdata-Update-%VER%" ^
  --add-data "%SHIP%\rtkdata-fw.bin;." ^
  --add-data "%SHIP%\ota_data_initial.bin;." ^
  --collect-all esptool ^
  --collect-all serial ^
  "%~dp0customer_update.py"

echo.
echo Built: %ROOT%\dist\RTKdata-Update-%VER%.exe
echo Test it on a spare device BEFORE sending it to anyone.
endlocal
