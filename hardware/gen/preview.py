"""PCB onizleme: padler, gövdeler, yollar, via'lar ve (yoksa) ratsnest.

    python3 preview.py <kart.kicad_pcb> <cikti.png> [--no-ratsnest]
"""

import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import os as _os
_os.environ.setdefault("KICAD7_FOOTPRINT_DIR", "/usr/share/kicad/footprints")
_os.environ.setdefault("KICAD7_SYMBOL_DIR", "/usr/share/kicad/symbols")
import pcbnew  # noqa: E402

TOP, BOT = '#c8553d', '#2a78d6'


def mm(v):
    return pcbnew.ToMM(v)


def main(path, out, ratsnest=True):
    b = pcbnew.LoadBoard(path)
    fig, ax = plt.subplots(figsize=(14, 11))
    ax.set_facecolor('#1d2a22')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            s, e = d.GetStart(), d.GetEnd()
            ax.plot([mm(s.x), mm(e.x)], [mm(s.y), mm(e.y)], color='#e8e0b0', lw=1)

    for z in b.Zones():
        poly = z.GetFilledPolysList(z.GetLayer()) if z.IsFilled() else None
        if poly is None:
            continue
        col = TOP if z.GetLayer() == pcbnew.F_Cu else BOT
        for i in range(poly.OutlineCount()):
            o = poly.Outline(i)
            xs = [mm(o.CPoint(k).x) for k in range(o.PointCount())]
            ys = [mm(o.CPoint(k).y) for k in range(o.PointCount())]
            ax.fill(xs, ys, color=col, alpha=0.12, lw=0)

    for t in b.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            p = t.GetPosition()
            ax.add_patch(mp.Circle((mm(p.x), mm(p.y)), mm(t.GetWidth()) / 2,
                                   color='#d9d9d9', zorder=6))
            continue
        s, e = t.GetStart(), t.GetEnd()
        col = TOP if t.GetLayer() == pcbnew.F_Cu else BOT
        ax.plot([mm(s.x), mm(e.x)], [mm(s.y), mm(e.y)], color=col,
                lw=max(mm(t.GetWidth()) * 72 / 25.4 * 11 / 85 * 6, 0.6),
                solid_capstyle='round', alpha=0.9, zorder=4 if col == TOP else 3)

    pads_by_net = {}
    for fp in b.GetFootprints():
        cy = fp.GetBoundingBox(False, False)
        ax.add_patch(mp.Rectangle((mm(cy.GetX()), mm(cy.GetY())), mm(cy.GetWidth()),
                                  mm(cy.GetHeight()), fill=False, ec='#8a8a7a', lw=0.5))
        c = fp.GetPosition()
        ax.text(mm(c.x), mm(c.y), fp.GetReference(), color='white', fontsize=6,
                ha='center', va='center', zorder=10)
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            smd = pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
            col = TOP if smd else '#d4a017'
            ax.add_patch(mp.Rectangle((mm(bb.GetX()), mm(bb.GetY())), mm(bb.GetWidth()),
                                      mm(bb.GetHeight()), color=col, alpha=0.8, zorder=5))
            if pad.GetNetname():
                p = pad.GetPosition()
                pads_by_net.setdefault(pad.GetNetname(), []).append((mm(p.x), mm(p.y)))

    if ratsnest:
        for net, pts in pads_by_net.items():
            if net == 'GND' or len(pts) < 2:
                continue
            done = [pts[0]]
            rest = pts[1:]
            while rest:
                best = min(((a, r) for a in done for r in rest),
                           key=lambda ar: math.dist(*ar))
                a, r = best
                ax.plot([a[0], r[0]], [a[1], r[1]], color='#f2e85c', lw=0.4, alpha=0.7)
                done.append(r)
                rest.remove(r)

    ax.autoscale_view()
    fig.savefig(out, dpi=110, bbox_inches='tight', facecolor='#1d2a22')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], '--no-ratsnest' not in sys.argv)
