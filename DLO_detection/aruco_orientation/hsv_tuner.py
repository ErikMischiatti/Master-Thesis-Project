# hsv_tuner.py
# This script provides a graphical interface with trackbars to adjust HSV values 
# for detecting connectors and cables using ArUco markers. It allows for real-time 
# tuning of parameters, with the option to save them to a JSON file.

import cv2
import numpy as np
from aruco_config import save_params, load_params
from aruco_realsense import RealSenseCamera

# Load existing parameters from the JSON file
params_file = '/home/.../catkin_ws/src/DLO_detection/scripts/params_v2.json'  # modify the path
params = load_params(params_file)

# default values 
if params is None:
    params = {
        'LH_Connector': 0, 'LS_Connector': 0, 'LV_Connector': 0,
        'UH_Connector': 179, 'US_Connector': 255, 'UV_Connector': 255,
        'LH_Cable': 0, 'LS_Cable': 0, 'LV_Cable': 0,
        'UH_Cable': 179, 'US_Cable': 255, 'UV_Cable': 255
    }

# Debug: SPrint the parameters loaded from the JSON file
print("Parametri caricati:")
for key, value in params.items():
    print(f"{key}: {value}")

def update_params(x):
    params['LH_Connector'] = cv2.getTrackbarPos('LH Connector', 'Trackbars')
    params['LS_Connector'] = cv2.getTrackbarPos('LS Connector', 'Trackbars')
    params['LV_Connector'] = cv2.getTrackbarPos('LV Connector', 'Trackbars')
    params['UH_Connector'] = cv2.getTrackbarPos('UH Connector', 'Trackbars')
    params['US_Connector'] = cv2.getTrackbarPos('US Connector', 'Trackbars')
    params['UV_Connector'] = cv2.getTrackbarPos('UV Connector', 'Trackbars')

    params['LH_Cable'] = cv2.getTrackbarPos('LH Cable', 'Trackbars')
    params['LS_Cable'] = cv2.getTrackbarPos('LS Cable', 'Trackbars')
    params['LV_Cable'] = cv2.getTrackbarPos('LV Cable', 'Trackbars')
    params['UH_Cable'] = cv2.getTrackbarPos('UH Cable', 'Trackbars')
    params['US_Cable'] = cv2.getTrackbarPos('US Cable', 'Trackbars')
    params['UV_Cable'] = cv2.getTrackbarPos('UV Cable', 'Trackbars')

# Create a window with trackbars to adjust HSV parameters
cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Trackbars', 500, 400)

# Create trackbars for connector values
cv2.createTrackbar('LH Connector', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('LS Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('LV Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UH Connector', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('US Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UV Connector', 'Trackbars', 0, 255, update_params)

# Create trackbars for cable values
cv2.createTrackbar('LH Cable', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('LS Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('LV Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UH Cable', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('US Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UV Cable', 'Trackbars', 0, 255, update_params)


trackbar_params = {
    'LH Connector': params['LH_Connector'],
    'LS Connector': params['LS_Connector'],
    'LV Connector': params['LV_Connector'],
    'UH Connector': params['UH_Connector'],
    'US Connector': params['US_Connector'],
    'UV Connector': params['UV_Connector'],
    'LH Cable': params['LH_Cable'],
    'LS Cable': params['LS_Cable'],
    'LV Cable': params['LV_Cable'],
    'UH Cable': params['UH_Cable'],
    'US Cable': params['US_Cable'],
    'UV Cable': params['UV_Cable']
}

for name, value in trackbar_params.items():
    cv2.setTrackbarPos(name, 'Trackbars', value)

camera = RealSenseCamera()

while True:
    color_frame, depth_frame = camera.get_frames()
    if color_frame is None or depth_frame is None:
        continue

    # Convert the frame to OpnCV format 
    color_image = np.asanyarray(color_frame.get_data())
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Apply HSV masks for the connector 
    lower_connector = np.array([params['LH_Connector'], params['LS_Connector'], params['LV_Connector']])
    upper_connector = np.array([params['UH_Connector'], params['US_Connector'], params['UV_Connector']])
    mask_connector = cv2.inRange(hsv_image, lower_connector, upper_connector)

    # Apply HSV masks for the cable 
    lower_cable = np.array([params['LH_Cable'], params['LS_Cable'], params['LV_Cable']])
    upper_cable = np.array([params['UH_Cable'], params['US_Cable'], params['UV_Cable']])
    mask_cable = cv2.inRange(hsv_image, lower_cable, upper_cable)

    cv2.imshow('Color Frame', color_image)
    cv2.imshow('Connector Mask', mask_connector)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'): 
        save_params(params, params_file)
        print(f"Parametri salvati in {params_file}")
    elif key == ord('q'): 
        break

cv2.destroyAllWindows()
camera.stop()
