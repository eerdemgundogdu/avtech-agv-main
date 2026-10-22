#!/usr/bin/env python3
# ============================================================================
# AVTECH AGV - HARITALAMA (MAPPING) ALGORITMASI TEST KODU
#
# Bu kod, Jetson uzerinde gercek zamanli calisacak olan 'mapping_node.py' 
# dosyasindaki algoritmalarin (Odometri + Engel Isleme) dogrulugunu
# hicbir donanima ihtiyac duymadan bilgisayar uzerinde test eder.
#
# KULLANIM:
# python3 jetson_mapping_test.py
# ============================================================================

import time
import sys
import os

# mapping_node.py ile ayni dizinde olmali
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from mapping_node import MappingNode
except ImportError:
    print("[HATA] mapping_node.py bulunamadi! Bu dosyayi jetson_software klasorunde calistirin.")
    sys.exit(1)

def print_map(mapper, size=15):
    """
    Robotun etrafindaki bolgeyi terminale cizer.
    Robot = 'R', Engel = '#', Bos = '.', Bilinmeyen = ' '
    """
    rx, ry = mapper._world_to_grid(mapper.pos_x, mapper.pos_y)
    n = mapper.grid_cells

    print("\n--- HARITA DURUMU ---")
    for gy in range(ry + size, ry - size, -1):
        row = ""
        for gx in range(rx - size, rx + size):
            if gx == rx and gy == ry:
                row += "R "
            elif not (0 <= gx < n and 0 <= gy < n):
                row += "  "
            else:
                val = mapper.grid[gy, gx]
                row += "# " if val == 2 else ". " if val == 1 else "  "
        print(row)
    print("---------------------\n")


def drive(mapper, left_rpm, right_rpm, yaw_deg, front, left, right, steps=10):
    """Sabit hiz ve sensor degerleriyle birkac adim simule et."""
    for _ in range(steps):
        mapper.update_odometry(left_rpm, right_rpm, yaw_deg)
        mapper.update_obstacles(front, left, right)
        time.sleep(0.02)


def run_test():
    print("=====================================")
    print("HARITALAMA VE ODOMETRI TESTI BASLIYOR")
    print("=====================================")

    mapper = MappingNode(grid_size_m=5.0, resolution_m=0.05)  # 5 cm cozunurluk
    start = mapper.get_position()

    print("[TEST 1] Baslangic noktasi:")
    print_map(mapper, size=10)

    print("[TEST 2] Robot kuzeye duz ilerliyor, sensorler bos.")
    drive(mapper, 50, 50, 0, 400, 400, 400)
    print_map(mapper, size=10)

    print("[TEST 3] Onde 40 cm'de engel.")
    drive(mapper, 0, 0, 0, 40, 400, 400, steps=3)
    print_map(mapper, size=10)

    print("[TEST 4] Robot doguya donup ilerliyor, solunda 20 cm'de engel.")
    drive(mapper, 50, 50, 90, 400, 20, 400)
    print_map(mapper, size=10)

    end = mapper.get_position()
    moved = abs(end['x'] - start['x']) + abs(end['y'] - start['y'])
    obstacles = len(mapper.get_map_data()['obstacles'])
    print(f"Konum: {end}  Engel hucresi: {obstacles}")
    assert moved > 0, "Odometri konumu guncellemedi"
    assert obstacles > 0, "Engel haritaya islenmedi"
    print("Test tamamlandi.")


if __name__ == "__main__":
    run_test()
