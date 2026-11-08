@echo off
chcp 65001 >nul
set ROOT=%~dp0
echo.
echo 正在恢复被 CubeMX 覆盖的 4 个文件...
copy /Y "%ROOT%restore\EWARM\yuancheng.ewp" "%ROOT%EWARM\yuancheng.ewp" >nul
copy /Y "%ROOT%restore\Core\Inc\stm32f7xx_hal_conf.h" "%ROOT%Core\Inc\stm32f7xx_hal_conf.h" >nul
copy /Y "%ROOT%restore\Core\Src\stm32f7xx_it.c" "%ROOT%Core\Src\stm32f7xx_it.c" >nul
copy /Y "%ROOT%restore\LWIP\Target\ethernetif.c" "%ROOT%LWIP\Target\ethernetif.c" >nul
echo.
echo 完成。请用 IAR 打开 EWARM\Project.eww，执行 Rebuild All，然后下载。
echo.
pause
