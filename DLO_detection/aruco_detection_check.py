import pyrealsense2 as rs
import cv2
import numpy as np

def main():
    # Configura il flusso della telecamera RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Inizia il flusso
    pipeline.start(config)
    
    try:
        while True:
            # Prendi i fotogrammi dalla telecamera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            # Controlla se il frame è valido
            if not color_frame:
                continue

            # Converti l'immagine in formato numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Rilevazione degli ArUco markers
            # Definisci il dizionario degli ArUco marker che vuoi rilevare
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            aruco_params = cv2.aruco.DetectorParameters()

            # Crea il rilevatore ArUco
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
            # Rileva i markers
            corners, ids, rejected = detector.detectMarkers(color_image)
            
            # Se vengono trovati markers, disegna i bordi
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            
            # Mostra il frame con i markers rilevati
            cv2.imshow('RealSense ArUco Detection', color_image)
            
            # Premi 'q' per uscire
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Ferma il flusso della telecamera e chiudi le finestre
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()