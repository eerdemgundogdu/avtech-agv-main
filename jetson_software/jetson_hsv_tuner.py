#!/usr/bin/env python3
# ============================================================================
# AVTECH AGV - KAMERA HSV RENK KALIBRASYON ARACI
#
# Yarisma alanindaki isiklandirma (floresan, gunes isigi vb.) kameranin
# renkleri algilamasini etkiler. Bu script ile cizgilerin (Kirmizi, Mavi, 
# Yesil, Sari) veya siyah bantlarin HSV alt/ust sinirlarini canli 
# olarak deneyerek bulabilirsiniz.
# 
# KULLANIM:
# python3 jetson_hsv_tuner.py
# ============================================================================

import cv2
import numpy as np

def nothing(x):
    pass

# Baslangic degerleri
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
cv2.resizeWindow('HSV Tuner', 600, 300)

# Trackbarlar
cv2.createTrackbar('HMin', 'HSV Tuner', 0, 179, nothing)
cv2.createTrackbar('SMin', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('VMin', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('HMax', 'HSV Tuner', 179, 179, nothing)
cv2.createTrackbar('SMax', 'HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('VMax', 'HSV Tuner', 255, 255, nothing)

# Ornek default deger (Mavi icin bir baslangic)
cv2.setTrackbarPos('HMin', 'HSV Tuner', 100)
cv2.setTrackbarPos('SMin', 'HSV Tuner', 80)
cv2.setTrackbarPos('VMin', 'HSV Tuner', 60)
cv2.setTrackbarPos('HMax', 'HSV Tuner', 130)
cv2.setTrackbarPos('SMax', 'HSV Tuner', 255)
cv2.setTrackbarPos('VMax', 'HSV Tuner', 255)

print("==================================================")
print("HSV KALIBRASYON ARACI BASLADI")
print("1. Kamerayi renkli cizgiye dogru tutun.")
print("2. Kaydiricilari oynatarak ekranda SADECE cizgiyi")
print("   BEYAZ, geri kalan her seyi SIYAH yapmaya calisin.")
print("3. Cikmak icin 'q' tusuna basin.")
print("4. Buldugunuz degerleri camera_node.py dosyasindaki")
print("   COLOR_RANGES kismina kaydedin.")
print("==================================================")

try:
    while True:
        ret, frame = cam.read()
        if not ret:
            break

        # Gorsel gürültüyü azalt (ana koddaki gibi)
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Trackbar degerlerini oku
        hMin = cv2.getTrackbarPos('HMin', 'HSV Tuner')
        sMin = cv2.getTrackbarPos('SMin', 'HSV Tuner')
        vMin = cv2.getTrackbarPos('VMin', 'HSV Tuner')
        hMax = cv2.getTrackbarPos('HMax', 'HSV Tuner')
        sMax = cv2.getTrackbarPos('SMax', 'HSV Tuner')
        vMax = cv2.getTrackbarPos('VMax', 'HSV Tuner')

        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Maske olustur
        mask = cv2.inRange(hsv, lower, upper)

        # Sonucu orjinal renkle goster
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Ekrana bas
        cv2.imshow('Orijinal Kamera', frame)
        cv2.imshow('Filtrelenmis (Maske)', mask)
        cv2.imshow('Renk Sonucu', result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[SONUC] Buldugunuz Degerler:")
            print(f"LOWER: np.array([{hMin}, {sMin}, {vMin}])")
            print(f"UPPER: np.array([{hMax}, {sMax}, {vMax}])")
            print("Bu degerleri camera_node.py -> COLOR_RANGES sozlugunde kullanabilirsiniz.")
            break

except KeyboardInterrupt:
    pass

cam.release()
cv2.destroyAllWindows()
