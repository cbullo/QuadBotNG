#!/bin/bash
FILE=../../mcu_firmware/leg_hex.hex
IFS='
'
for x in $(yq e '.controllers.*.controller_address' ../config/robot_config.yaml); do
  avrdude -c arduino -p m328p -D -v -V -U flash:w:${FILE}:i -U flash:v:${FILE}:i -b 57600 -P $x
done