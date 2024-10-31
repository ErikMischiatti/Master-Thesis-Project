#!/bin/bash

echo "Waiting for the camera to initialize..."
sleep 2  # Waits 2 seconds; adjust as needed

echo "Setting camera parameters..."

# Turn off auto exposure
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1

# Turn off auto focus
v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video2 --set-ctrl=focus_automatic_continuous=0


echo "Camera parameters set."