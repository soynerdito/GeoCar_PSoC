@REM This script allows a 3rd party IDE to use CyHexTool to perform
@REM any post processing that is necessary to convert the raw flash
@REM image into a complete hex file to use in programming the PSoC.
@REM USAGE: post_link.bat
@REM    arg1: Persistant path back to the directory containing the app project.
@REM    arg2: Path (relative to arg1) of the directory where the hex files go.
@REM    arg3: Name of the project.
@REM NOTE: This script is auto generated. Do not modify.

"C:\Program Files\Cypress\PSoC Creator\2.2\PSoC Creator\bin\cyvalidateide.exe" -dev CY8C5868LTI-LP039 -ide "%~1\%~3" -flsAddr 0x2800 -flsSize 0x3D7C0 -sramAddr 0x1FFF8000 -sramSize 0x10000
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
move "%~1\%~2\%~n3.hex" "%~1\%~2\%~n3.ihx"
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
"C:\Program Files\Cypress\PSoC Creator\2.2\PSoC Creator\bin\cysymboladdressfinder.exe" -f "%~1\%~2\%~n3.axf" -s Reset -p "%~1\%~2\entryaddress.txt" -l 0 
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
"C:\Program Files\Cypress\PSoC Creator\2.2\PSoC Creator\bin\cyhextool" -o "%~1\%~2\%~n3.hex" -f "%~1\%~2\%~n3.ihx" -flsLine 256 -arraySize 65536 -id 2E127069 -rev 0 -ecc "%~dp0config.hex" -metaRow 0 -blver 00020000000000000000 -chksumEcc -blChkType 0 -endian l -a EEPROM=90200000:800,PROGRAM=00000000:40000,CONFIG=80000000:8000,PROTECT=90400000:100 -acd -acdStart 2800 -e "%~1\%~2\entryaddress.txt" 
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
"C:\Program Files\Cypress\PSoC Creator\2.2\PSoC Creator\bin\cyhextool" -o "%~1\%~2\%~n3.hex" -flsLine 256 -arraySize 65536 -bl ..\..\..\..\..\..\..\Program Files\Cypress\CY8CKIT-042 PSoC 4 Pioneer Kit\1.0\Firmware\Programmer\KitProg_Bootloader\KitProg_Bootloader.hex -blver 00020000000000000000 -chksumEcc -blChkType 0 -endian l -prot "%~dp0protect.hex" -id 2E127069 -a EEPROM=90200000:800,PROGRAM=00000000:40000,CONFIG=80000000:8000,PROTECT=90400000:100 -meta 0001 -cunv 00004007 -wonv BC90ACAF -ecc "%~dp0config.hex" -ld "%~1\%~2\%~n3.ihx=2800:3D7BF" -acdStart 2800 -e "%~1\%~2\entryaddress.txt" 
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
CD /D "C:\Keil\UV4"
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
IF EXIST "C:\Users\wbz\Desktop\Element14 Posts\036 - Booting the PSoC 5LP Device - NS\Bootloading_PSoC5_LP\Bootloading_PSoC5_LP.cydsn\Bootloading_PSoC5_LP.svd" "C:\Keil\UV4\SVDConv.exe" "C:\Users\wbz\Desktop\Element14 Posts\036 - Booting the PSoC 5LP Device - NS\Bootloading_PSoC5_LP\Bootloading_PSoC5_LP.cydsn\Bootloading_PSoC5_LP.svd" -o "C:\Users\wbz\Desktop\Element14 Posts\036 - Booting the PSoC 5LP Device - NS\Bootloading_PSoC5_LP\Bootloading_PSoC5_LP.cydsn" --generate=sfr -n "Bootloading_PSoC5_LP"
IF NOT EXIST "C:\Users\wbz\Desktop\Element14 Posts\036 - Booting the PSoC 5LP Device - NS\Bootloading_PSoC5_LP\Bootloading_PSoC5_LP.cydsn\Bootloading_PSoC5_LP.svd" rem "C:\Users\wbz\Desktop\Element14 Posts\036 - Booting the PSoC 5LP Device - NS\Bootloading_PSoC5_LP\Bootloading_PSoC5_LP.cydsn\Bootloading_PSoC5_LP.sfr"
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
