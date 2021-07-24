#!/bin/bash
IFS='
'
for x in `yq r robot_config.yaml controllers.*.controller_address`; do 
avrdude -c arduino -p $$1 -D -V -U flash:w:$$2:i -U flash:v:$$2:i -b 57600 -P $x
done