#!/bin/bash

video_device="$1"

if [ -z "$video_device" ]; then
  echo "No video device specified."
  exit 1
fi

if [ ! -e "$video_device" ]; then
  echo "Video device $video_device not found."
  exit 1
fi

echo "Using video device: $video_device"

echo "Waiting for the camera to initialize..."
sleep 2  # Waits 2 seconds; adjust as needed

echo "Setting camera parameters..."

# Turn off auto exposure
v4l2-ctl -d "$video_device" --set-ctrl=auto_exposure=1

# Turn off auto focus
v4l2-ctl -d "$video_device" --set-ctrl=focus_automatic_continuous=0
