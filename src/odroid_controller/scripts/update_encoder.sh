#!/bin/bash
set -x

encoder_offset=`yq e '.encoders.'$1'.linearization_offset' ../config/robot_config.yaml`
encoder_cooefs=`yq e '.encoders.'$1'.linearization_coeffs' ../config/robot_config.yaml`
encoder_controller=`yq e '.encoders.'$1'.controller' ../config/robot_config.yaml`
encoder_cooefs=${encoder_cooefs:1:-1}
controller_address=`yq e '.controllers.'$encoder_controller'.controller_address' ../config/robot_config.yaml`

stty 115200 -F $controller_address raw -echo
sleep 1
echo "EO$encoder_offset">$controller_address
sleep 0.1
IFS=', ' read -r -a cooefs_values <<< "$encoder_cooefs"
for i in "${!cooefs_values[@]}"
do
    printf "EC%02d%d\n" $i ${cooefs_values[$i]}>$controller_address
    sleep 0.1
done 
