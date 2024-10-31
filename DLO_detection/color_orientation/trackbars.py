import cv2

def nothing(x):
    pass

def setup_trackbars():
    """Setup OpenCV trackbars for HSV parameters."""
    cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
    sections = ['Connector', 'Cable', 'Top', 'Bottom', 'Left', 'Right']
    for section in sections:
        cv2.createTrackbar(f'LH {section}', 'Parameters', 0, 179, nothing)
        cv2.createTrackbar(f'LS {section}', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar(f'LV {section}', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar(f'UH {section}', 'Parameters', 179, 179, nothing)
        cv2.createTrackbar(f'US {section}', 'Parameters', 255, 255, nothing)
        cv2.createTrackbar(f'UV {section}', 'Parameters', 255, 255, nothing)

def load_saved_params(params):
    """Load saved HSV parameters into trackbars."""
    sections = ['Connector', 'Cable', 'Top', 'Bottom', 'Left', 'Right']
    if params:
        for section in sections:
            cv2.setTrackbarPos(f'LH {section}', 'Parameters', params[f'LH_{section}'])
            cv2.setTrackbarPos(f'LS {section}', 'Parameters', params[f'LS_{section}'])
            cv2.setTrackbarPos(f'LV {section}', 'Parameters', params[f'LV_{section}'])
            cv2.setTrackbarPos(f'UH {section}', 'Parameters', params[f'UH_{section}'])
            cv2.setTrackbarPos(f'US {section}', 'Parameters', params[f'US_{section}'])
            cv2.setTrackbarPos(f'UV {section}', 'Parameters', params[f'UV_{section}'])

def get_trackbar_values():
    """Retrieve current HSV values from trackbars."""
    values = {}
    sections = ['Connector', 'Cable', 'Top', 'Bottom', 'Left', 'Right']
    for section in sections:
        values[f'l_h_{section.lower()}'] = cv2.getTrackbarPos(f'LH {section}', 'Parameters')
        values[f'l_s_{section.lower()}'] = cv2.getTrackbarPos(f'LS {section}', 'Parameters')
        values[f'l_v_{section.lower()}'] = cv2.getTrackbarPos(f'LV {section}', 'Parameters')
        values[f'u_h_{section.lower()}'] = cv2.getTrackbarPos(f'UH {section}', 'Parameters')
        values[f'u_s_{section.lower()}'] = cv2.getTrackbarPos(f'US {section}', 'Parameters')
        values[f'u_v_{section.lower()}'] = cv2.getTrackbarPos(f'UV {section}', 'Parameters')
    return values