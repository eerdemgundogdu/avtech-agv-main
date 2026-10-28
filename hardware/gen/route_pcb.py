"""Yerlesimi kurar, tum netleri yonlendirir ve kaydeder.

    python3 route_pcb.py
"""

import os
import sys
import time

import os as _os
_os.environ.setdefault('KICAD7_FOOTPRINT_DIR', '/usr/share/kicad/footprints')
import pcbnew

import make_pcb
import router


def attempt(priority, verbose):
    board = make_pcb.build()
    grid = router.Grid(board, make_pcb.ORIGIN_X, make_pcb.ORIGIN_Y,
                       make_pcb.BOARD_W, make_pcb.BOARD_H, make_pcb.CORNER_R)
    r = router.Router(grid)
    nets = sorted(grid.pads.keys())
    order = router.net_order(grid, nets)
    # Onceki denemede basarisiz olan netler (GND haric) one alinir
    order = [n for n in priority if n in order and n != 'GND'] + \
            [n for n in order if n not in priority or n == 'GND']
    for n in order:
        t = time.time()
        ok = r.route_net(n)
        if verbose and not ok:
            print(f'  HATA {n:16s} {time.time() - t:5.1f}s', flush=True)
    return board, r


def run(save_name='routed.kicad_pcb', verbose=True, max_attempts=4):
    t0 = time.time()
    priority, best = [], None
    for k in range(max_attempts):
        board, r = attempt(priority, verbose)
        print(f'deneme {k + 1}: {len(r.failed)} basarisiz', flush=True)
        if best is None or len(r.failed) < len(best[1].failed):
            best = (board, r)
        if not r.failed:
            break
        for net, _ in r.failed:
            if net not in priority:
                priority.insert(0, net)
    board, r = best
    # Parca govdelerinin altina via koyma (lehim/montaj icin)
    keepout = []
    for fp in board.GetFootprints():
        bb = fp.GetBoundingBox(False, False)
        keepout.append((pcbnew.ToMM(bb.GetLeft()) - make_pcb.ORIGIN_X - 0.5,
                        pcbnew.ToMM(bb.GetTop()) - make_pcb.ORIGIN_Y - 0.5,
                        pcbnew.ToMM(bb.GetRight()) - make_pcb.ORIGIN_X + 0.5,
                        pcbnew.ToMM(bb.GetBottom()) - make_pcb.ORIGIN_Y + 0.5))
    r.stitch_count = r.add_stitching('GND', 4.0, keepout)
    print(f'{r.stitch_count} GND dikis via', flush=True)
    r.apply(board)
    out = make_pcb.save(board, save_name)
    print(f'{len(r.tracks)} iz, {len(r.vias)} via, {len(r.failed)} basarisiz baglanti, '
          f'{time.time() - t0:.0f}s')
    for f in r.failed:
        print('  BASARISIZ', f)
    for n in r.necked:
        print('  INCELTILDI', n)
    return out, r


if __name__ == '__main__':
    run()
