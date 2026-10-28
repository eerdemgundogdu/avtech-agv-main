"""Basit iki katmanli izgara yonlendirici (A*, 45/90 derece).

Her net, once guc netleri olmak uzere sirayla yonlendirilir. Bir net icin
diger netlerin bakiri (iz yari genisligi + bosluk) kadar sisirilip engel
yapilir; A* durumu (katman, y, x, yon) uzerinde calisir, donuslere ve
via'lara maliyet ekler. Sonuc KiCad DRC'si ile ayrica dogrulanir.
"""

import heapq
import math

import numpy as np
import pcbnew
from scipy import ndimage

RES = 0.127                     # izgara (mm)
CLR_RULE = 0.2                  # bakir-bakir bosluk kurali (mm)
# Izgara bakiri hucre merkezine gore yuvarlar; gercek kenar yarim hucre
# (RES * 0.71) kadar tasabilir. Yonlendirme bu payla yapilir.
CLR = CLR_RULE + RES * 0.75
EDGE_CLR = 0.5 + RES * 0.75     # bakir-kart kenari (mm) + izgara payi
VIA_D, VIA_DRILL = 0.6, 0.3
VIA_COST = 2.5                  # mm esdegeri
BOTTOM_MULT = 1.6               # alt katmani GND duzlemine birakmak icin
TURN45, TURN90 = 0.15, 0.6      # mm esdegeri donus cezalari
H_WEIGHT = 1.25                 # agirlikli A*: hedefe yonelir, yol en fazla %25 uzar
MAX_EXPANSIONS = 6_000_000

WIDTHS = {
    'VBAT_IN': 0.8, 'VBAT_F': 0.8, '+24V': 0.8, 'BUCK_SW': 0.8,
    '+5V': 0.6, 'ESTOP_RET': 0.6, 'COIL_N': 0.6,
    'GND': 0.5, 'TEENSY_VIN': 0.5, '+3V3': 0.4,
}
DEFAULT_W = 0.25
NECK_W = 0.3                    # ince pinlere girerken iz bu genislige iner
NECK_ZONE = 1.0                 # neck-down'a izin verilen ped cevresi (mm)

DIRS = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
LAYERS = [pcbnew.F_Cu, pcbnew.B_Cu]


def mm(v):
    return pcbnew.FromMM(float(v))


class Grid:
    def __init__(self, board, ox, oy, w, h, corner_r):
        self.board, self.ox, self.oy = board, ox, oy
        self.W = int(math.ceil(w / RES)) + 1
        self.H = int(math.ceil(h / RES)) + 1
        self.owner = np.zeros((2, self.H, self.W), dtype=np.int32)
        self.padowner = np.zeros((2, self.H, self.W), dtype=np.int32)   # sadece pedler
        self.netid, self.netname = {}, {}
        self.netinfo = {}
        for name, ni in board.GetNetsByName().items():
            name = str(name)
            if not name:
                continue
            k = len(self.netid) + 1
            self.netid[name] = k
            self.netname[k] = name
            self.netinfo[name] = ni
        yy, xx = np.mgrid[0:self.H, 0:self.W]
        self.X, self.Y = xx * RES, yy * RES
        # kart disi = -1 (yuvarlak koseler dahil)
        outside = (self.X < 0) | (self.Y < 0) | (self.X > w) | (self.Y > h)
        for cx, cy in [(corner_r, corner_r), (w - corner_r, corner_r),
                       (corner_r, h - corner_r), (w - corner_r, h - corner_r)]:
            inx = (np.abs(self.X - cx) < corner_r + 1e-9) & ((self.X - cx) * (cx - w / 2) > 0)
            iny = (np.abs(self.Y - cy) < corner_r + 1e-9) & ((self.Y - cy) * (cy - h / 2) > 0)
            outside |= inx & iny & ((self.X - cx) ** 2 + (self.Y - cy) ** 2 > corner_r ** 2)
        # kenara en yakin hucreler de kenar sayilir (sisirme bundan yapilir)
        edge = outside.copy()
        edge[0, :] = edge[-1, :] = True
        edge[:, 0] = edge[:, -1] = True
        self.edge = edge
        self.pads = {}      # net -> [(layers, cells mask idx, center)]
        self._raster_pads()

    # --- koordinat donusumleri -------------------------------------------
    def to_local(self, p):
        return pcbnew.ToMM(p.x) - self.ox, pcbnew.ToMM(p.y) - self.oy

    def cell(self, x, y):
        return int(round(y / RES)), int(round(x / RES))

    def to_vec(self, i, j):
        return pcbnew.VECTOR2I(mm(self.ox + j * RES), mm(self.oy + i * RES))

    # --- rasterlama --------------------------------------------------------
    def _win(self, x0, y0, x1, y1):
        i0 = max(int(math.floor(y0 / RES)) - 1, 0)
        i1 = min(int(math.ceil(y1 / RES)) + 2, self.H)
        j0 = max(int(math.floor(x0 / RES)) - 1, 0)
        j1 = min(int(math.ceil(x1 / RES)) + 2, self.W)
        return i0, i1, j0, j1

    def pad_mask(self, pad, shrink=0.0):
        bb = pad.GetBoundingBox()
        x0, y0 = self.to_local(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop()))
        x1, y1 = self.to_local(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
        i0, i1, j0, j1 = self._win(x0, y0, x1, y1)
        X, Y = self.X[i0:i1, j0:j1], self.Y[i0:i1, j0:j1]
        shape = pad.GetShape()
        if shape in (pcbnew.PAD_SHAPE_CIRCLE,) or pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            r = (x1 - x0) / 2 - shrink
            m = (X - cx) ** 2 + (Y - cy) ** 2 <= r * r
        elif shape == pcbnew.PAD_SHAPE_OVAL:
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            a, b = (x1 - x0) / 2 - shrink, (y1 - y0) / 2 - shrink
            rr = min(a, b)
            dx = np.maximum(np.abs(X - cx) - (a - rr), 0)
            dy = np.maximum(np.abs(Y - cy) - (b - rr), 0)
            m = dx * dx + dy * dy <= rr * rr
        else:
            m = ((X >= x0 + shrink) & (X <= x1 - shrink) &
                 (Y >= y0 + shrink) & (Y <= y1 - shrink))
        return (i0, i1, j0, j1), m

    def _raster_pads(self):
        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                net = pad.GetNetname()
                nid = self.netid.get(net, -2) if net else -2
                (i0, i1, j0, j1), m = self.pad_mask(pad)
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    # delik + ekstra pay: iki katmanda da engel
                    (i0, i1, j0, j1), m = self.pad_mask(pad, shrink=-0.3)
                for li, L in enumerate(LAYERS):
                    if pad.IsOnLayer(L) or pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        sub = self.owner[li, i0:i1, j0:j1]
                        sub[m] = nid
                        self.padowner[li, i0:i1, j0:j1][m] = nid
                if net:
                    (ti0, ti1, tj0, tj1), tm = self.pad_mask(pad, shrink=0.05)
                    layers = [li for li, L in enumerate(LAYERS) if pad.IsOnLayer(L)]
                    c = self.to_local(pad.GetPosition())
                    self.pads.setdefault(net, []).append(
                        (layers, (ti0, tj0, tm), c, fp.GetReference() + '.' + pad.GetNumber()))

    def mark_segment(self, li, x0, y0, x1, y1, hw, nid):
        i0, i1, j0, j1 = self._win(min(x0, x1) - hw, min(y0, y1) - hw,
                                   max(x0, x1) + hw, max(y0, y1) + hw)
        X, Y = self.X[i0:i1, j0:j1], self.Y[i0:i1, j0:j1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 == 0:
            d2 = (X - x0) ** 2 + (Y - y0) ** 2
        else:
            t = np.clip(((X - x0) * dx + (Y - y0) * dy) / L2, 0, 1)
            d2 = (X - x0 - t * dx) ** 2 + (Y - y0 - t * dy) ** 2
        sub = self.owner[li, i0:i1, j0:j1]
        sub[(d2 <= hw * hw) & (sub == 0)] = nid


def disk(r):
    y, x = np.mgrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r


class Router:
    def __init__(self, grid):
        self.g = grid
        self.tracks = []    # (layer, x0, y0, x1, y1, w, net)
        self.vias = []      # (x, y, net)
        self.failed = []
        self.necked = []    # (net, ped, istenen genislik)

    def blocked(self, nid, hw, win):
        i0, i1, j0, j1 = win
        rt = int(math.ceil((hw + CLR) / RES - 1e-6))
        re = int(math.ceil((hw + EDGE_CLR) / RES - 1e-6))
        rv = int(math.ceil((VIA_D / 2 + CLR) / RES - 1e-6))
        rve = int(math.ceil((VIA_D / 2 + EDGE_CLR) / RES - 1e-6))
        pad = max(rt, re, rv, rve) + 1
        a0, a1 = max(i0 - pad, 0), min(i1 + pad, self.g.H)
        b0, b1 = max(j0 - pad, 0), min(j1 + pad, self.g.W)
        sl = (slice(i0 - a0, i0 - a0 + (i1 - i0)), slice(j0 - b0, j0 - b0 + (j1 - j0)))
        edge = self.g.edge[a0:a1, b0:b1]
        e_t = ndimage.binary_dilation(edge, disk(re))[sl]
        e_v = ndimage.binary_dilation(edge, disk(rve))[sl]
        bt, bv = [], []
        for li in range(2):
            o = self.g.owner[li, a0:a1, b0:b1]
            other = (o != 0) & (o != nid)
            bt.append(ndimage.binary_dilation(other, disk(rt))[sl] | e_t)
            bv.append(ndimage.binary_dilation(other, disk(rv))[sl] | e_v)
        via_ok = ~(bv[0] | bv[1])
        return np.stack(bt), via_ok

    def route_net(self, net):
        g = self.g
        pads = g.pads.get(net, [])
        if len(pads) < 2:
            return True
        nid = g.netid[net]
        w = WIDTHS.get(net, DEFAULT_W)
        hw = w / 2
        # Prim benzeri: en yakin bagli olmayan pede baglan
        order = [pads[0]]
        rest = pads[1:]
        tree_pts = [(l, pads[0][2]) for l in pads[0][0]]
        tree_cells = self._pad_cells(pads[0])
        ok = True
        while rest:
            tgt = min(rest, key=lambda p: min(math.dist(p[2], c) for _, c in tree_pts))
            rest.remove(tgt)
            path = self._astar(nid, hw, tree_cells, tgt, net)
            cw = w
            if path is None and w > NECK_W:
                # Kalin iz pinler arasina sigmiyorsa bu dali ince dene
                # (ornek: JST XH pinleri arasi 0.8 mm). Kullanim: dusuk akimli dallar.
                path = self._astar(nid, NECK_W / 2, tree_cells, tgt, net)
                cw = NECK_W
                if path is not None:
                    self.necked.append((net, tgt[3], w))
            if path is None:
                self.failed.append((net, tgt[3]))
                ok = False
                tree_cells |= self._pad_cells(tgt)
                continue
            segs = self._commit(path, nid, cw, net, tgt)
            for (li, i, j) in path:
                tree_cells.add((li, i, j))
            tree_cells |= self._pad_cells(tgt)
            tree_pts.extend((l, tgt[2]) for l in tgt[0])
            tree_pts.extend((li, (j * RES, i * RES)) for li, i, j in path[::8])
        return ok

    def _pad_cells(self, pad):
        layers, (ti0, tj0, tm), _, _ = pad
        cells = set()
        ii, jj = np.nonzero(tm)
        for li in layers:
            for a, b in zip(ii, jj):
                cells.add((li, ti0 + a, tj0 + b))
        return cells

    def _astar(self, nid, hw, sources, tgt, net):
        # Once kaynak/hedef cevresindeki pencerede ara (hizli); yol yoksa
        # (ornek: Teensy'nin ucundan dolasmak gerekiyorsa) tum kartta ara.
        path = self._astar_win(nid, hw, sources, tgt, net, full=False)
        if path is None:
            path = self._astar_win(nid, hw, sources, tgt, net, full=True)
        return path

    def _astar_win(self, nid, hw, sources, tgt, net, full):
        g = self.g
        tcells = self._pad_cells(tgt)
        allc = list(sources) + list(tcells)
        if full:
            i0, i1, j0, j1 = 0, g.H, 0, g.W
        else:
            mi = 8.0 / RES
            i0 = max(int(min(c[1] for c in allc) - mi), 0)
            i1 = min(int(max(c[1] for c in allc) + mi) + 1, g.H)
            j0 = max(int(min(c[2] for c in allc) - mi), 0)
            j1 = min(int(max(c[2] for c in allc) + mi) + 1, g.W)
        bt, via_ok = self.blocked(nid, hw, (i0, i1, j0, j1))
        nhw = min(hw, NECK_W / 2)
        btn = bt if nhw == hw else self.blocked(nid, nhw, (i0, i1, j0, j1))[0]
        Hh, Ww = i1 - i0, j1 - j0
        # Kendi bakirimiz (ped/iz) icinde iz ince cizilir (neck-down); orada
        # sadece ince iz icin bosluk aranir.
        own = [(g.owner[li, i0:i1, j0:j1] == nid) for li in range(2)]
        rz = int(math.ceil(NECK_ZONE / RES))
        zone = [own[li] | ndimage.binary_dilation(g.padowner[li, i0:i1, j0:j1] == nid, disk(rz))
                for li in range(2)]
        free = np.stack([~bt[li] | (zone[li] & ~btn[li]) for li in range(2)])
        # bu hucrede kalin iz kurala uymuyorsa ince cizilecek
        self._narrow = (i0, j0, np.stack([zone[li] & bt[li] for li in range(2)]))
        tset = set()
        for (li, i, j) in tcells:
            if i0 <= i < i1 and j0 <= j < j1:
                tset.add((li, i - i0, j - j0))
        if not tset:
            return None
        tx, ty = tgt[2]
        tci, tcj = ty / RES - i0, tx / RES - j0
        ND = 9
        N = 2 * Hh * Ww * ND
        gcost = np.full(N, np.inf, dtype=np.float64)
        parent = np.full(N, -1, dtype=np.int64)

        def idx(li, i, j, d):
            return ((li * Hh + i) * Ww + j) * ND + d

        def h(i, j):
            dy, dx = abs(i - tci), abs(j - tcj)
            return (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)) * RES * H_WEIGHT

        heap = []
        for (li, i, j) in sources:
            if not (i0 <= i < i1 and j0 <= j < j1):
                continue
            ii, jj = i - i0, j - j0
            if not free[li, ii, jj]:
                continue
            s = idx(li, ii, jj, 8)
            gcost[s] = 0
            heapq.heappush(heap, (h(ii, jj), 0.0, s))
        step = [RES * (math.sqrt(2) if d % 2 else 1) for d in range(8)]
        lm = [1.0, BOTTOM_MULT]
        found = None
        expansions = 0
        while heap:
            f, gc, s = heapq.heappop(heap)
            if gc > gcost[s]:
                continue
            d = s % ND
            rem = s // ND
            j = rem % Ww
            rem //= Ww
            i = rem % Hh
            li = rem // Hh
            if (li, i, j) in tset:
                found = s
                break
            expansions += 1
            if expansions > MAX_EXPANSIONS:
                break
            for nd, (di, dj) in enumerate(DIRS):
                if d != 8:
                    dd = min((nd - d) % 8, (d - nd) % 8)
                    if dd > 2:
                        continue
                    tc = 0 if dd == 0 else (TURN45 if dd == 1 else TURN90)
                else:
                    tc = 0
                ni, nj = i + di, j + dj
                if not (0 <= ni < Hh and 0 <= nj < Ww) or not free[li, ni, nj]:
                    continue
                if nd % 2 and not (free[li, i + di, j] and free[li, i, j + dj]):
                    continue
                ng = gc + step[nd] * lm[li] + tc
                t = idx(li, ni, nj, nd)
                if ng < gcost[t]:
                    gcost[t] = ng
                    parent[t] = s
                    heapq.heappush(heap, (ng + h(ni, nj), ng, t))
            if via_ok[i, j]:
                ol = 1 - li
                if free[ol, i, j]:
                    ng = gc + VIA_COST
                    t = idx(ol, i, j, 8)
                    if ng < gcost[t]:
                        gcost[t] = ng
                        parent[t] = s
                        heapq.heappush(heap, (ng + h(i, j), ng, t))
        if found is None:
            return None
        path = []
        s = found
        while s != -1:
            rem = s // ND
            j = rem % Ww
            rem //= Ww
            i = rem % Hh
            li = rem // Hh
            path.append((li, i + i0, j + j0))
            s = int(parent[s])
        path.reverse()
        return path

    def _commit(self, path, nid, w, net, tgt):
        g = self.g
        nw = min(w, NECK_W)
        ni0, nj0, narrow = self._narrow

        def is_narrow(c):
            li, i, j = c
            return bool(narrow[li, i - ni0, j - nj0])

        flags = [is_narrow(c) for c in path]
        # Kirilma noktalari: katman, yon veya daralma degisimi
        pts = [0]
        for k in range(1, len(path) - 1):
            a, b, c = path[k - 1], path[k], path[k + 1]
            if b[0] != a[0] or c[0] != b[0] or flags[k] != flags[k + 1]:
                pts.append(k)
                continue
            if (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2]):
                pts.append(k)
        if len(path) > 1:
            pts.append(len(path) - 1)
        drawn = []
        for a_k, b_k in zip(pts, pts[1:]):
            a, b = path[a_k], path[b_k]
            if a[0] != b[0]:
                x, y = a[2] * RES, a[1] * RES
                self.vias.append((x, y, net))
                for li in range(2):
                    g.mark_segment(li, x, y, x, y, VIA_D / 2, nid)
                continue
            if (a[1], a[2]) == (b[1], b[2]):
                continue
            sw = nw if any(flags[a_k:b_k + 1]) else w
            drawn.append((a[0], a[2] * RES, a[1] * RES, b[2] * RES, b[1] * RES, sw))
        last = path[-1]
        tx, ty = tgt[2]
        if last[0] in tgt[0]:
            # son nokta ped icinde; ped merkezine ince bir kisa parca
            drawn.append((last[0], last[2] * RES, last[1] * RES, tx, ty, nw))
        for (li, x0, y0, x1, y1, sw) in drawn:
            self.tracks.append((li, x0, y0, x1, y1, sw, net))
            g.mark_segment(li, x0, y0, x1, y1, sw / 2, nid)
        return drawn

    def add_stitching(self, net='GND', pitch=4.0, keepout=None):
        """Bos alanlara duzenli GND via'lari: ust ve alt dokumu baglar,
        izlerle cevrelenmis bakir adaciklarini kurtarir."""
        g = self.g
        nid = g.netid[net]
        _, via_ok = self.blocked(nid, VIA_D / 2, (0, g.H, 0, g.W))
        # ayrica kendi bakirimizdan da (GND pedleri/izleri) bir via boyu uzak dur
        n = 0
        step = int(round(pitch / RES))
        for i in range(step // 2, g.H, step):
            for j in range(step // 2, g.W, step):
                if not via_ok[i, j]:
                    continue
                x, y = j * RES, i * RES
                if keepout and any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in keepout):
                    continue
                self.vias.append((x, y, net))
                for li in range(2):
                    g.mark_segment(li, x, y, x, y, VIA_D / 2, nid)
                n += 1
        return n

    def apply(self, board):
        g = self.g
        for (li, x0, y0, x1, y1, w, net) in self.tracks:
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(g.ox + x0), mm(g.oy + y0)))
            t.SetEnd(pcbnew.VECTOR2I(mm(g.ox + x1), mm(g.oy + y1)))
            t.SetWidth(mm(w))
            t.SetLayer(LAYERS[li])
            t.SetNet(g.netinfo[net])
            board.Add(t)
        for (x, y, net) in self.vias:
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(mm(g.ox + x), mm(g.oy + y)))
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNet(g.netinfo[net])
            board.Add(v)


def hpwl(grid, net):
    pts = [p[2] for p in grid.pads.get(net, [])]
    if len(pts) < 2:
        return 0
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (max(xs) - min(xs)) + (max(ys) - min(ys))


def net_order(grid, nets):
    power = [n for n in nets if n in WIDTHS and n != 'GND']
    sig = [n for n in nets if n not in WIDTHS]
    power.sort(key=lambda n: hpwl(grid, n))
    sig.sort(key=lambda n: hpwl(grid, n))
    tail = ['GND'] if 'GND' in nets else []
    return power + sig + tail
