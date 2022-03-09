@echo off
echo Flash Slave
avrdude -C avrdude.conf -c avrispmkII -P usb -p t261 -U flash:w:firmware.hex:a -v
PAUSE