@echo off
setlocal

:: Change this to your ESP32 toolchain path if needed
set PATH=C:\Users\%USERNAME%\.platformio\packages\toolchain-xtensa-esp32\bin;%PATH%

:: Set your IDF path or the path where espcoredump.py is located
set IDF_PATH=C:\Users\%USERNAME%\.platformio\packages\framework-arduinoespressif32\tools\esp-idf

:: Set the path to your ELF file and coredump file
set ELF_FILE=build\firmware.elf
set COREDUMP_FILE=coredump.bin

:: Optional: allow passing arguments
if not "%~1"=="" set ELF_FILE=%~1
if not "%~2"=="" set COREDUMP_FILE=%~2

:: Run the decoding
python "%IDF_PATH%\components\espcoredump\espcoredump.py" info_corefile --core %COREDUMP_FILE% --core-format raw --elf %ELF_FILE%

pause
