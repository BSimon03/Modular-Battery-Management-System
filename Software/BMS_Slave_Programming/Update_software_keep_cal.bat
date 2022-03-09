@echo off
echo Flash Slave with updated BMS firmware
cd avrdude
::avrdude -c avrispmkII  -p attiny261 -U eeprom:r:../callibration_data:x.hex:i
::timeout 2
avrdude -c avrispmkII  -p attiny261 -U flash:w:../firmware.hex:i
timeout 2
avrdude -c avrispmkII  -p attiny261 -U flash:v:../firmware.hex:i
::timeout 2
::avrdude -c avrispmkII  -p attiny261 -U eeprom:w:../callibration_data.hex:i
PAUSE