"""Yonlendirme teshisi: tum netleri sirayla cizer, basarisiz her baglanti icin
hedef pedin cevresindeki engel haritasini PNG olarak kaydeder.

    python3 debug_route.py <cikti_klasoru>
"""

import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import make_pcb  # noqa: E402
import router  # noqa: E402


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    board = make_pcb.build()
    g = router.Grid(board, make_pcb.ORIGIN_X, make_pcb.ORIGIN_Y,
                    make_pcb.BOARD_W, make_pcb.BOARD_H, make_pcb.CORNER_R)
    r = router.Router(g)
    order = router.net_order(g, sorted(g.pads.keys()))
    done = 0
    for n in order:
        before = len(r.failed)
        r.route_net(n)
        for net, pad in r.failed[before:]:
            tgt = next(p for p in g.pads[net] if p[3] == pad)
            cx, cy = tgt[2]
            ci, cj = int(cy / router.RES), int(cx / router.RES)
            span = int(6 / router.RES)
            win = (max(ci - span, 0), min(ci + span, g.H), max(cj - span, 0), min(cj + span, g.W))
            nid = g.netid[net]
            bt, via_ok = r.blocked(nid, min(router.WIDTHS.get(net, router.DEFAULT_W), router.NECK_W) / 2, win)
            fig, axs = plt.subplots(1, 3, figsize=(15, 5))
            for li in range(2):
                own = g.owner[li, win[0]:win[1], win[2]:win[3]] == nid
                img = np.zeros(bt[li].shape + (3,))
                img[bt[li]] = (0.6, 0.15, 0.15)
                img[own] = (0.2, 0.7, 0.2)
                axs[li].imshow(img, origin='upper')
                axs[li].set_title(f'{net} -> {pad}  {"F.Cu" if li == 0 else "B.Cu"}  (kirmizi=engel, yesil=kendi)')
                axs[li].plot(cj - win[2], ci - win[0], 'y+', ms=15)
            axs[2].imshow(via_ok, origin='upper', cmap='gray')
            axs[2].set_title('via konabilir (beyaz)')
            f = os.path.join(outdir, f'fail_{net.replace("+", "p")}_{pad}.png')
            fig.savefig(f, dpi=70, bbox_inches='tight')
            plt.close(fig)
            print('kaydedildi', f, flush=True)
            done += 1
    print('basarisiz:', r.failed)


if __name__ == '__main__':
    main(sys.argv[1])
