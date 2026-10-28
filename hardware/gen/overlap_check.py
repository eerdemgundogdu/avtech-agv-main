"""Pad bazli kaba yerlesim cakisma kontrolu (DRC'den once hizli geri bildirim)."""
import itertools
import sys

import os as _os
_os.environ.setdefault("KICAD7_FOOTPRINT_DIR", "/usr/share/kicad/footprints")
_os.environ.setdefault("KICAD7_SYMBOL_DIR", "/usr/share/kicad/symbols")
import pcbnew


def boxes(path):
    b = pcbnew.LoadBoard(path)
    out = []
    for fp in b.GetFootprints():
        xs0, ys0, xs1, ys1 = [], [], [], []
        items = list(fp.Pads())
        for pad in items:
            pb = pad.GetBoundingBox()
            xs0.append(pb.GetLeft()); ys0.append(pb.GetTop())
            xs1.append(pb.GetRight()); ys1.append(pb.GetBottom())
        if not items:
            pb = fp.GetBoundingBox(False, False)
            xs0, ys0, xs1, ys1 = [pb.GetLeft()], [pb.GetTop()], [pb.GetRight()], [pb.GetBottom()]
        out.append((fp.GetReference(), pcbnew.ToMM(min(xs0)), pcbnew.ToMM(min(ys0)),
                    pcbnew.ToMM(max(xs1)), pcbnew.ToMM(max(ys1))))
    return out


def main(path, margin=0.3):
    n = 0
    for a, c in itertools.combinations(boxes(path), 2):
        if a[1] - margin < c[3] and c[1] - margin < a[3] and a[2] - margin < c[4] and c[2] - margin < a[4]:
            print('CAKISMA', a[0], c[0])
            n += 1
    print(f'{n} cakisma')


if __name__ == '__main__':
    main(sys.argv[1])
