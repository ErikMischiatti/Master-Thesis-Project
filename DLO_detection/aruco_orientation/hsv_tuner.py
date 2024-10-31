import cv2
import numpy as np
from aruco_config import save_params, load_params
from aruco_realsense import RealSenseCamera

# Carica i parametri esistenti dal file JSON
params_file = '/home/asl_team/catkin_ws/src/DLO_detection/scripts/params_v2.json'
params = load_params(params_file)

# Se non ci sono parametri, usa dei valori di default
if params is None:
    params = {
        'LH_Connector': 0, 'LS_Connector': 0, 'LV_Connector': 0,
        'UH_Connector': 179, 'US_Connector': 255, 'UV_Connector': 255,
        'LH_Cable': 0, 'LS_Cable': 0, 'LV_Cable': 0,
        'UH_Cable': 179, 'US_Cable': 255, 'UV_Cable': 255
    }

# Debug: Stampa i valori caricati dal file JSON
print("Parametri caricati:")
for key, value in params.items():
    print(f"{key}: {value}")

# Funzione per aggiornare i valori dalle trackbars
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

# Crea una finestra con trackbars per regolare i parametri HSV
cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Trackbars', 500, 400)

# Crea le trackbars per i valori del connettore
cv2.createTrackbar('LH Connector', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('LS Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('LV Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UH Connector', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('US Connector', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UV Connector', 'Trackbars', 0, 255, update_params)

# Crea le trackbars per i valori del cavo
cv2.createTrackbar('LH Cable', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('LS Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('LV Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UH Cable', 'Trackbars', 0, 179, update_params)
cv2.createTrackbar('US Cable', 'Trackbars', 0, 255, update_params)
cv2.createTrackbar('UV Cable', 'Trackbars', 0, 255, update_params)

# Imposta i valori delle trackbars con un ciclo
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

# Ciclo per impostare i valori delle trackbars
for name, value in trackbar_params.items():
    cv2.setTrackbarPos(name, 'Trackbars', value)

# # Debug: Stampa i valori impostati nelle trackbars
# for name in trackbar_params:
#     print(f"Trackbar {name}: {cv2.getTrackbarPos(name, 'Trackbars')}")

# Inizializza la videocamera RealSense
camera = RealSenseCamera()

# Ciclo principale per mostrare i frame e regolare i parametri
while True:
    # Ottieni i frame dalla videocamera
    color_frame, depth_frame = camera.get_frames()
    if color_frame is None or depth_frame is None:
        continue

    # Converti il frame in formato OpenCV
    color_image = np.asanyarray(color_frame.get_data())
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Applica le maschere HSV per il connettore
    lower_connector = np.array([params['LH_Connector'], params['LS_Connector'], params['LV_Connector']])
    upper_connector = np.array([params['UH_Connector'], params['US_Connector'], params['UV_Connector']])
    mask_connector = cv2.inRange(hsv_image, lower_connector, upper_connector)

    # Applica le maschere HSV per il cavo
    lower_cable = np.array([params['LH_Cable'], params['LS_Cable'], params['LV_Cable']])
    upper_cable = np.array([params['UH_Cable'], params['US_Cable'], params['UV_Cable']])
    mask_cable = cv2.inRange(hsv_image, lower_cable, upper_cable)

    # Mostra i frame originali e le maschere
    cv2.imshow('Color Frame', color_image)
    cv2.imshow('Connector Mask', mask_connector)
    # cv2.imshow('Cable Mask', mask_cable)

    # Controlla i tasti premuti
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # Se si preme 's', salva i parametri
        save_params(params, params_file)
        print(f"Parametri salvati in {params_file}")
    elif key == ord('q'):  # Se si preme 'q', esce dallo script
        break

# Chiudi tutte le finestre e ferma la videocamera
cv2.destroyAllWindows()
camera.stop()
