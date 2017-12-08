@echo off
setlocal enabledelayedexpansion

rem tools folder level
set WLEVEL=..\..\..
set LLEVEL=../../..

rem Product
set sname=ism43362
set mcu=stm32f2x
set fsize=1048576

rem Check command line
if [%1] == [] goto error
goto start

:error
rem Input error
echo.
echo Usage: !sname!_write_flash_image {file name}
echo        ex. !sname!_write_flash_image !sname!_m3g_l44_flash_image.bin
echo.
goto cleanup

:start
rem File name
set fname=%1
echo.
echo Writing !fname!

rem Write flash image
!WLEVEL!\tools\OpenOCD\Win32\openocd-all-brcm-libftdi.exe -f !LLEVEL!/tools/OpenOCD/BCM9WCD1EVAL1.cfg -f !LLEVEL!/tools/OpenOCD/!mcu!.cfg -f !LLEVEL!/tools/OpenOCD/!mcu!-flash-app.cfg -c "flash write_image erase {!fname!} 0x08000000 bin" -c shutdown > write.log 2>&1

:cleanup
rem Erase varaibles
set mcu=
set fsize=
set fname=
set WLEVEL=
set LLEVEL=

echo Press or issue a reset to module to start
