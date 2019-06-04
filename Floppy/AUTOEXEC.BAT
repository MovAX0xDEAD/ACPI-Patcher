@echo off
set PATH=R:\FD0\DOS
A:\DOS\shsurdrv /D:16384K:D512:$RAM-DRIVE,R /QQ
md R:FD0
md R:FD1
A:\DOS\xcopy A: R:\FD0 /E >nul
A:\DOS\xcopy B: R:\FD1 /E >nul

set COMSPEC=R:\FD0\COMMAND.COM
command.com /P:R:\FD0\autoexe2.bat
