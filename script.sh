#!/bin/bash

ros2 run cyclosafe gpio

if [ $? -eq 1 ]; then
    echo "System shutdown"
	sleep 1
    sudo halt
fi
echo "Error: $?"
