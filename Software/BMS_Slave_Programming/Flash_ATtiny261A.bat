@echo off
echo Flash Slave
cd avrdude
avrdude -c avrispmkII -p attiny261 -U flash:w:../firmware.hex:i
PAUSE