#!/bin/bash
TARGET_ADDRESS=robot@192.168.0.12

#ssh ${TARGET_ADDRESS} "cd /home/robot/control/odroid_controller; ./controller"
ssh ${TARGET_ADDRESS} "sudo systemctl start robot"