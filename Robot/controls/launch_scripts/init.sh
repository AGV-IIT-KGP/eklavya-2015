#!/bin/bash

echo bone_eqep1 > /sys/devices/bone_capemgr.*/slots
echo bone_eqep0 > /sys/devices/bone_capemgr.*/slots
sudo xboxdrv
