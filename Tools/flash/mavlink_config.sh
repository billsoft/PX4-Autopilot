#!/bin/sh
# MAVLink configuration script for NSH
mavlink status
mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 120
echo "ATTITUDE_QUATERNION stream rate set to 120 Hz"
