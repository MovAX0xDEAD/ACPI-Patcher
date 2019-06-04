@echo off
set PATH=R:\FD0\DOS;R:\FD1
set COMSPEC=R:\FD0\COMMAND.COM
set TMP=R:\
set TEMP=R:\

R:
cd FD1

7zdec.exe e R:\FD0\exe.7z > nul
a.exe
if errorlevel 1 goto shell
grub.exe

:shell
vc.com
