@echo off
echo Flashing Slave with callibration firmware
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U flash:w:firmware.hex:i
timeout 5
echo Adjust to 3V
timeout 60
echo Adjust to 4V
timeout 60
echo Extract callibration data
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U eeprom:r:callibration_data.hex:i
timeout 5
echo Flash Slave with BMS firmware
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U flash:w:firmware.hex:i
timeout 5
echo Write Callibration data into eeprom
::avrdude -P COM1 -c avrispmkII  -p attiny261 -U eeprom:w:callibration_data.hex:i
PAUSE