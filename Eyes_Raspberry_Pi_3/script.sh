#!/bin/bash

counter=1
while true 
do
    scrot pics/$counter.png
    sleep 10
    ((counter++))
done 
