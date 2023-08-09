cd /d %~dp0
:: C:\Keil\ARM\ARMCC\bin\fromelf --bin --output .\project.bin  .\project.axf
C:\Keil\ARM\ARMCC\bin\fromelf --text -a -c --output=.\project.lst  .\project.axf

copy .\Objects\project.axf project.axf
copy .\Objects\project.hex project.hex
copy .\Listings\project.map project.map

BinScript.exe BinScript.BinScript

makecode.exe

BinScript.exe BinScript_app_hex.BinScript
