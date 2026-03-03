#!/bin/bash

source /opt/ros/humble/setup.bash

export LOG_LEVEL=1

# Vstup do složky
cd "/mnt/c/Users/admin/OneDrive - VUT/Plocha/skola/ctvrtak letni/RBT/workplace/build"

# spusteni
./sender_node "/mnt/c/Users/admin/OneDrive - VUT/Plocha/skola/ctvrtak letni/RBT/workplace/config.json"


