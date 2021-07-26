#!/bin/bash

RUNFILES=${BASH_SOURCE[0]}.runfiles
DATA_FILES=$RUNFILES/__main__/src

rsync -irvzPLt --exclude=odroid_controller/scripts/deploy.sh ${DATA_FILES}/ robot@192.168.0.12:/home/robot/control
#ssh robot@192.168.0.12 "cd /home/robot/control/odroid_controller/scripts; ./upload_firmware.sh"
