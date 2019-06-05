# ACPI-Patcher

Prepare DEV environment:
1) Install base DJGPP packages in DOS (VM, DOSBOX, ...)
2) Install GCC compiler 4.x.x in DJGPP

Compiling:

    gcc ini.c main.c -o a.exe

Prepare Image:
1) Update a.exe inside exe.7z
2) Update exe.7z inside acpi_pat.img
3) Update acpi_pat.img on OS drive
