@echo off
echo Flash Slave with updated BMS firmware
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U eeprom:r:callibration_data.hex:i
timeout 2
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U flash:w:firmware.hex:i
timeout 2
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U eeprom:w:callibration_data.hex:i
PAUSE