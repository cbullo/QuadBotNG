#!/bin/bash
set -e

RUNFILES=${BASH_SOURCE[0]}.runfiles
DATA_FILES=$RUNFILES/__main__/src

TARGET_ADDRESS=robot@192.168.0.12

ssh ${TARGET_ADDRESS} "sudo systemctl stop robot"

RSYNC_OUTPUT=$(rsync -irvzPLt --exclude=odroid_controller/scripts/deploy.sh ${DATA_FILES}/ ${TARGET_ADDRESS}:/home/robot/control)
if [[ $RSYNC_OUTPUT == *"mcu_firmware/leg_hex.hex"* ]]; then
  ssh ${TARGET_ADDRESS} "cd /home/robot/control/odroid_controller/scripts; ./upload_firmware.sh"
fi

ssh ${TARGET_ADDRESS} "sudo systemctl start robot"
