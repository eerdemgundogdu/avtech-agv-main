#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import json
import os
import argparse

def calibrate_camera(image_dir, grid_size=(9, 6), square_size_cm=2.5, show_images=False):
    """
    Kamera kalibrasyonu için checkerboard resimlerini işler ve sonuçları kaydeder.

    Args:
        image_dir: Satranç tahtası fotoğraflarının bulunduğu dizin
        grid_size: Satranç tahtasındaki İÇ KÖŞE sayısı (sütun, satır)
        square_size_cm: Bir karenin fiziksel boyutu (cm)
        show_images: Köşeleri göster (True/False)
    """
    print(f"Kalibrasyon başlatılıyor... Dizin: {image_dir}")
    print(f"Grid boyutu: {grid_size}, Kare boyutu: {square_size_cm}cm")

    # Sub-pixel köşe bulma kriterleri
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 3D uzaydaki objenin gerçek koordinatları (Z=0)
    # Örn: (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    objp = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)
    objp *= square_size_cm

    # Tüm resimler için toplanacak nokta dizileri
    objpoints = [] # 3D noktalar (gerçek dünya)
    imgpoints = [] # 2D noktalar (görüntü düzlemi)

    images = glob.glob(os.path.join(image_dir, '*.jpg'))
    images += glob.glob(os.path.join(image_dir, '*.png'))

    if not images:
        print("HATA: Klasörde görüntü bulunamadı!")
        return False

    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]

        # Satranç tahtası köşelerini bul
        ret, corners = cv2.findChessboardCorners(gray, grid_size, None)

        if ret:
            objpoints.append(objp)
            # Daha hassas (sub-pixel) köşe koordinatları
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            if show_images:
                cv2.drawChessboardCorners(img, grid_size, corners2, ret)
                cv2.imshow('Calibration', img)
                cv2.waitKey(500)
        else:
            print(f"UYARI: {fname} dosyasında satranç tahtası bulunamadı.")

    if show_images:
        cv2.destroyAllWindows()

    if len(objpoints) == 0:
        print("HATA: Başarılı tespit edilen satranç tahtası yok.")
        return False

    print(f"Toplam {len(objpoints)} görüntü başarıyla işlendi. Kalibrasyon hesaplanıyor...")

    # Kalibrasyon hesaplama
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)

    # Re-projection hatasını hesapla (Kalitenin göstergesi)
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    avg_error = mean_error / len(objpoints)
    print(f"Kalibrasyon tamamlandı. Ortalama Hata (Reprojection Error): {avg_error:.4f} piksel")
    print("\nKamera Matrisi (Intrinsic):\n", mtx)
    print("\nBozulma Katsayıları (Distortion):\n", dist)

    # Sonuçları JSON olarak kaydet
    calib_data = {
        "camera_matrix": mtx.tolist(),
        "dist_coeffs": dist.tolist(),
        "image_width": img_shape[0],
        "image_height": img_shape[1],
        "reprojection_error": avg_error
    }

    out_file = 'camera_calib.json'
    with open(out_file, 'w') as f:
        json.dump(calib_data, f, indent=4)

    print(f"\nKalibrasyon verileri '{out_file}' dosyasına kaydedildi.")
    print("qr_reader.py bu dosyayı okuyarak hassas pozisyon tahmini yapabilir.")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Satranç Tahtası (Checkerboard) Kamera Kalibrasyonu')
    parser.add_argument('--dir', type=str, default='calib_images', help='Görüntülerin bulunduğu klasör')
    parser.add_argument('--grid', type=str, default='9x6', help='İç köşe sayısı (örnek: 9x6)')
    parser.add_argument('--size', type=float, default=2.5, help='Satranç karesinin boyutu (cm)')
    parser.add_argument('--show', action='store_true', help='İşlenen resimleri göster')

    args = parser.parse_args()

    grid = tuple(map(int, args.grid.split('x')))

    if not os.path.exists(args.dir):
        print(f"Klasör bulunamadı: {args.dir}")
        print("Lütfen satranç tahtası fotoğraflarını çekip bu klasöre koyun.")
        os.makedirs(args.dir, exist_ok=True)
    else:
        calibrate_camera(args.dir, grid_size=grid, square_size_cm=args.size, show_images=args.show)
