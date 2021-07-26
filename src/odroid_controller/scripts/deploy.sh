#!/bin/bash

RUNFILES=${BASH_SOURCE[0]}.runfiles
DATA_FILES=$RUNFILES/__main__/src

rsync -rvzPL --exclude=odroid_controller/scripts/deploy.sh ${DATA_FILES}/ robot@192.168.0.12:/home/robot/control