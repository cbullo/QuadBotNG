#!/bin/bash
set -x
IFS='
'

if [ $2 = "all" ]; then
  controllers=`yq e '.controllers.*.controller_address' ../config/robot_config.yaml`
else
  controllers=$2
fi

for x in $controllers; do
  avrdude -c arduino -p m328p -n -D -v -v -V -U flash:v:$1:i -b 115200 -P $x
  exit_status=$?
  if [ $exit_status -eq 0 ]; then
    echo "Firmware equal, skipping"
  else   
    avrdude -c arduino -p m328p -D -v -v -V -U flash:w:$1:i -U flash:v:$1:i -b 115200 -P $x
  fi
done
