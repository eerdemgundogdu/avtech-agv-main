#!/usr/bin/env python3
# ============================================================================
#  AVTECH AGV — Odometri Tabanlı Haritalandırma
#  Motor encoder (hoverboard feedback) + IMU ile 2D grid harita oluşturur.
#  Ultrasonik sensör verilerinden engelleri haritaya işler.
# ============================================================================

import math
import json
import time
import numpy as np

class MappingNode:
    """Basit 2D occupancy grid haritası — odometri + ultrasonik tabanlı."""

    def __init__(self, grid_size_m=10.0, resolution_m=0.05):
        """
        Args:
            grid_size_m: Harita boyutu (metre) — kare alan
            resolution_m: Grid çözünürlüğü (metre/piksel)
        """
        self.grid_size = grid_size_m
        self.resolution = resolution_m
        self.grid_cells = int(grid_size_m / resolution_m)

        # Occupancy grid: 0=bilinmeyen, 1=boş, 2=engel
        self.grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.uint8)

        # Robot pozisyonu (metre)
        self.pos_x = grid_size_m / 2.0   # Haritanın ortasından başla
        self.pos_y = grid_size_m / 2.0
        self.heading = 0.0               # Derece (0=kuzey, 90=doğu)

        # Odometri parametreleri
        self.wheel_circumference_m = 0.165 * math.pi  # ~16.5cm tekerlek çapı
        self.wheel_base_m = 0.45                        # Tekerlekler arası mesafe
        self.rpm_to_mps = self.wheel_circumference_m / 60.0  # RPM → m/s çevirimi

        # İz (trajectory)
        self.trajectory = []
        self.trajectory.append((self.pos_x, self.pos_y))

        # Son güncelleme zamanı
        self.last_update_time = time.time()

    def update_odometry(self, speed_left_rpm, speed_right_rpm, yaw_deg):
        """Motor hızları ve IMU yaw açısından pozisyon güncelle.

        Args:
            speed_left_rpm: Sol motor hızı (RPM, hoverboard feedback)
            speed_right_rpm: Sağ motor hızı (RPM)
            yaw_deg: IMU yaw açısı (derece)
        """
        now = time.time()
        dt = now - self.last_update_time
        if dt > 1.0:
            dt = 0.1  # İlk çağrı veya uzun boşluk
        self.last_update_time = now

        # RPM → m/s
        v_left = speed_left_rpm * self.rpm_to_mps
        v_right = speed_right_rpm * self.rpm_to_mps

        # Ortalama hız ve dönüş
        v_linear = (v_left + v_right) / 2.0

        # Heading — IMU kullan (daha doğru)
        self.heading = yaw_deg

        # Pozisyon güncelle
        heading_rad = math.radians(self.heading)
        dx = v_linear * math.cos(heading_rad) * dt
        dy = v_linear * math.sin(heading_rad) * dt

        self.pos_x += dx
        self.pos_y += dy

        # Sınır kontrolü
        self.pos_x = max(0.0, min(self.pos_x, self.grid_size))
        self.pos_y = max(0.0, min(self.pos_y, self.grid_size))

        # Robot pozisyonunu boş olarak işaretle
        gx, gy = self._world_to_grid(self.pos_x, self.pos_y)
        if 0 <= gx < self.grid_cells and 0 <= gy < self.grid_cells:
            self.grid[gy][gx] = 1  # Boş

        # İz kaydet (her 10cm'de bir)
        if len(self.trajectory) == 0 or \
           math.sqrt((self.pos_x - self.trajectory[-1][0])**2 +
                     (self.pos_y - self.trajectory[-1][1])**2) > 0.1:
            self.trajectory.append((self.pos_x, self.pos_y))

    def update_obstacles(self, front_cm, left_cm, right_cm):
        """Ultrasonik sensör verilerinden engelleri haritalaya işle.

        Args:
            front_cm: Ön ultrasonik mesafe (cm)
            left_cm: Sol-ön ultrasonik mesafe (cm)
            right_cm: Sağ-ön ultrasonik mesafe (cm)
        """
        heading_rad = math.radians(self.heading)

        # Sensör açıları (robota göre)
        sensor_angles = [
            0.0,          # Ön: düz ileri
            -30.0,        # Sol-ön: -30°
            30.0           # Sağ-ön: +30°
        ]
        distances = [front_cm / 100.0, left_cm / 100.0, right_cm / 100.0]

        for i, (angle_offset, dist) in enumerate(zip(sensor_angles, distances)):
            if dist >= 2.0:  # 2m üzeri → geçerli engel yok
                continue

            # Engel pozisyonunu hesapla
            total_angle = heading_rad + math.radians(angle_offset)
            obs_x = self.pos_x + dist * math.cos(total_angle)
            obs_y = self.pos_y + dist * math.sin(total_angle)

            gx, gy = self._world_to_grid(obs_x, obs_y)
            if 0 <= gx < self.grid_cells and 0 <= gy < self.grid_cells:
                self.grid[gy][gx] = 2  # Engel

            # Sensör ile engel arasındaki hücreleri boş işaretle (ray-casting)
            self._raycast_free(self.pos_x, self.pos_y, obs_x, obs_y)

    def _world_to_grid(self, wx, wy):
        """Dünya koordinatını grid hücresine çevir."""
        gx = int(wx / self.resolution)
        gy = int(wy / self.resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        """Grid hücresini dünya koordinatına çevir."""
        wx = (gx + 0.5) * self.resolution
        wy = (gy + 0.5) * self.resolution
        return wx, wy

    def _raycast_free(self, x0, y0, x1, y1):
        """Robot ile engel arasındaki hücreleri boş işaretle."""
        gx0, gy0 = self._world_to_grid(x0, y0)
        gx1, gy1 = self._world_to_grid(x1, y1)

        # Basit Bresenham çizgi algoritması
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        while True:
            if 0 <= gx0 < self.grid_cells and 0 <= gy0 < self.grid_cells:
                if self.grid[gy0][gx0] == 0:  # Sadece bilinmeyen hücreleri güncelle
                    self.grid[gy0][gx0] = 1   # Boş

            if gx0 == gx1 and gy0 == gy1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx0 += sx
            if e2 < dx:
                err += dx
                gy0 += sy

    def get_map_data(self):
        """GUI'ye gönderilecek harita verisi (JSON formatında)."""
        # Grid'i sıkıştırılmış formatta gönder (sadece dolu hücreler)
        obstacles = []
        free_cells = []

        # Çözünürlüğü düşür (GUI için)
        scale = 4  # Her 4x4 hücreyi birleştir
        reduced_size = self.grid_cells // scale

        for gy in range(reduced_size):
            for gx in range(reduced_size):
                # 4x4 bölgenin modu
                region = self.grid[gy*scale:(gy+1)*scale, gx*scale:(gx+1)*scale]
                if np.any(region == 2):
                    obstacles.append([gx, gy])
                elif np.any(region == 1):
                    free_cells.append([gx, gy])

        # Trajectory'yi metre cinsinden
        traj = [(round(x, 2), round(y, 2)) for x, y in self.trajectory[-100:]]

        return {
            'grid_size': reduced_size,
            'resolution': self.resolution * scale,
            'robot_x': round(self.pos_x, 3),
            'robot_y': round(self.pos_y, 3),
            'heading': round(self.heading, 1),
            'obstacles': obstacles,
            'free': free_cells,
            'trajectory': traj
        }

    def get_position(self):
        """Robot pozisyonunu döndür."""
        return {
            'x': round(self.pos_x, 3),
            'y': round(self.pos_y, 3),
            'heading': round(self.heading, 1)
        }


# --- Test ---

if __name__ == '__main__':
    mapper = MappingNode(grid_size_m=5.0, resolution_m=0.05)

    # Simülasyon: düz ileri git
    for i in range(100):
        mapper.update_odometry(50, 50, 0)  # 50 RPM, kuzey yönü
        mapper.update_obstacles(100, 200, 200)  # Ön 1m, yanlar boş
        time.sleep(0.05)

    pos = mapper.get_position()
    print(f"Konum: X={pos['x']:.2f}m  Y={pos['y']:.2f}m  Heading={pos['heading']:.1f}°")

    map_data = mapper.get_map_data()
    print(f"Harita: {map_data['grid_size']}x{map_data['grid_size']} hücre")
    print(f"Engel sayısı: {len(map_data['obstacles'])}")
    print(f"İz noktası: {len(map_data['trajectory'])}")
