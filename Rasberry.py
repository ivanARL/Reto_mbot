import cv2
import numpy as np
import time
import serial
# === INICIALIZACIÓN DEL PUERTO SERIAL ===
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    print("Conexión con Arduino establecida.")
except serial.SerialException:
    arduino = None
    print("No se pudo abrir el puerto serial con Arduino.")
# === INICIALIZACIÓN DE LA CÁMARA ===
cap = cv2.VideoCapture(0)
# Parámetros para detección de líneas
rho = 2
theta = np.pi / 180
threshold = 40
min_line_len = 50
max_line_gap = 10
ultimo_tiempo = time.time()
intervalo_segundos = 0.1
imagen_con_lineas = None
while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame de la cámara.")
        break
    # === CONVERSIÓN A HSV Y DETECCIÓN DE AMARILLO ===
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # === APLICAR MÁSCARA AMARILLA A LA IMAGEN ===
    result = cv2.bitwise_and(frame, frame, mask=mask_yellow)
    # === ESCALA DE GRISES, DESENFOQUE Y CANNY ===
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(blur, 50, 150)
    # === MÁSCARA DE REGIÓN DE INTERÉS ===
    vertices = np.array([[[(10, 410), (10, 100), (630, 100), (630, 410)]]], dtype=np.int32)
    roi_mask = np.zeros_like(edges)
    cv2.fillPoly(roi_mask, vertices, 255)
    img_masked = cv2.bitwise_and(edges, roi_mask)
    tiempo_actual = time.time()
    if tiempo_actual - ultimo_tiempo >= intervalo_segundos:
        ultimo_tiempo = tiempo_actual
        img_mask_color = cv2.cvtColor(img_masked, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLinesP(img_masked, rho, theta, threshold, np.array([]),
                                minLineLength=min_line_len, maxLineGap=max_line_gap)
        angulos = []
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(img_mask_color, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    dx = x2 - x1
                    dy = y2 - y1
                    angle = 90.0 if dx == 0 else np.degrees(np.arctan2(dy, dx))
                    if angle < 0:
                        angle += 180
                    angulos.append(angle)
            promedio = np.mean(angulos)
            angulo_envio = 90  # Valor por defecto
            texto = ""
            if promedio == 90:
                texto = "Ángulo de conducción: 0° (recto)"
            elif promedio < 90:
                texto = f"Ángulo: {round(90 - promedio, 2)}° (izquierda)"
                angulo_envio = round(promedio)
            else:
                texto = f"Ángulo: {round(promedio - 90, 2)}° (derecha)"
                angulo_envio = round(promedio)
            cv2.putText(img_mask_color, texto, (30, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2, cv2.LINE_AA)
            # Enviar al Arduino
            if arduino:
                try:
                    arduino.write(f"{angulo_envio}\n".encode())
                    print(f"Ángulo enviado al Arduino: {angulo_envio}")
                except Exception as e:
                    print(f"Error al enviar al Arduino: {e}")
        else:
            cv2.putText(img_mask_color, "No se detectaron líneas", (30, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2, cv2.LINE_AA)
        imagen_con_lineas = img_mask_color.copy()
    if imagen_con_lineas is not None:
        cv2.imshow("Detección de línea amarilla y ángulo", imagen_con_lineas)
    if cv2.waitKey(30) == 13:  # Tecla Enter
        break
cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
