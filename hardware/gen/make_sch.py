"""design.py netlist'inden KiCad sematigi (.kicad_sch) uretir.

Her parca kendi fonksiyon blogunda yerlesir; baglantilar pin uclarina konan
net etiketleriyle yapilir (tel cizimi yok). Kullanilmayan pinlere
"no connect" isareti konur.
"""

import copy
import os
import uuid

from kiutils.items.common import (Effects, Font, Justify, PageSettings, Position,
                                  Property, TitleBlock)
from kiutils.items.schitems import (HierarchicalSheetInstance, LocalLabel,
                                    NoConnect, SchematicSymbol, SymbolInstance,
                                    Text)
from kiutils.schematic import Schematic
from kiutils.symbol import SymbolLib

import design

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.join(HERE, '..', 'avtech-agv-carrier')
SYMDIR = '/usr/share/kicad/symbols'
GRID = 2.54

BLOCK_TITLES = {
    'power': 'Guc girisi ve koruma (24 V)',
    'buck': '24 V -> 5 V buck (LMR16006X, 700 kHz)',
    'ldo': '5 V -> 3.3 V LDO ve Teensy beslemesi',
    'mcu': 'Teensy 4.1',
    'hoverboard': 'Hoverboard FOC UART',
    'imu': 'IMU (ICM-20948, SPI0)',
    'qtr': 'QTR-8A cizgi sensoru',
    'ultrasonic': 'HC-SR04 x3 (ECHO 5 V -> 3.0 V)',
    'stepper': 'DM556 step surucu (forklift)',
    'ui': 'Buzzer ve durum LED',
    'estop': 'Donanimsal acil stop ve motor gucu kontaktoru',
    'misc': 'Test noktalari ve montaj',
}
BLOCK_ORDER = ['power', 'buck', 'ldo', 'estop', 'mcu', 'hoverboard', 'imu',
               'qtr', 'ultrasonic', 'stepper', 'ui', 'misc']


def snap(v):
    return round(v / GRID) * GRID


def fx(v):
    return round(v, 4)


_lib_cache = {}


def load_symbol(lib_id):
    """Kutuphaneden sembolu alir; 'extends' ile turetilmisse duzlestirir."""
    lib, name = lib_id.split(':')
    path = (os.path.join(PROJ, 'avtech.kicad_sym') if lib == 'avtech'
            else os.path.join(SYMDIR, f'{lib}.kicad_sym'))
    if path not in _lib_cache:
        _lib_cache[path] = {s.entryName: s for s in SymbolLib.from_file(path).symbols}
    by = _lib_cache[path]
    sym = copy.deepcopy(by[name])
    if sym.extends:
        parent = copy.deepcopy(by[sym.extends])
        child_props = {p.key: p for p in sym.properties}
        props = []
        for p in parent.properties:
            props.append(child_props.pop(p.key, p))
        props.extend(child_props.values())
        parent.properties = props
        parent.entryName = name
        for u in parent.units:
            u.entryName = name
        sym = parent
        sym.extends = None
    sym.libraryNickname = lib
    return sym


def pins_of(sym):
    out = []
    for u in sym.units:
        for p in u.pins:
            out.append(p)
    return out


CHAR_W = 1.05      # 1.27 mm yazi icin yaklasik karakter genisligi


def label_len(net):
    return len(net) * CHAR_W + 2.54 if net else 1.27


def side_extents(part, sym):
    """Her yon icin pin etiketlerinin kapladigi en uzun mesafe (mm)."""
    ext = {0: 0.0, 180: 0.0, 90: 0.0, 270: 0.0}
    for q in pins_of(sym):
        ext[q.position.angle] = max(ext[q.position.angle],
                                    label_len(part.pins.get(q.number, '')))
    return ext


def cell_size(part, sym):
    pins = pins_of(sym)
    if not pins:
        return 15.24, 15.24
    xs = [p.position.X for p in pins]
    ys = [p.position.Y for p in pins]
    ext = side_extents(part, sym)
    right_free = ext[180] == 0
    text_w = 10.16 if right_free else 0
    cw = (max(xs) - min(xs)) + ext[0] + ext[180] + text_w + 7.62
    ch = (max(ys) - min(ys)) + ext[90] + ext[270] + (5.08 if right_free else 12.7) + 5.08
    return max(cw, 15.24), max(ch, 17.78)


def label_effects(angle):
    just = 'left' if angle in (0, 90) else 'right'
    return Effects(font=Font(width=1.27, height=1.27),
                   justify=Justify(horizontally=just))


def prop(key, value, x, y, hide=False, angle=0, left=False):
    return Property(key=key, value=value, id=None,
                    position=Position(X=fx(x), Y=fx(y), angle=angle),
                    effects=Effects(font=Font(width=1.27, height=1.27), hide=hide,
                                    justify=Justify(horizontally='left') if left else Justify()))


def build():
    sch = Schematic.create_new()
    sch.paper = PageSettings(paperSize='A2')
    sch.titleBlock = TitleBlock()
    sch.titleBlock.title = 'AVTECH AGV Tasiyici Kart'
    sch.titleBlock.revision = 'A'
    sch.titleBlock.date = '2026-10-28'
    sch.titleBlock.company = 'CU AVTECH'
    sch.titleBlock.comments = {
        1: 'Rev A: tasarim asamasinda, henuz uretilmedi ve test edilmedi',
        2: 'Kaynak: hardware/gen/design.py (bu dosya uretilir, elle duzenlemeyin)',
    }
    root = sch.uuid = str(uuid.uuid4())

    lib_syms = {}
    by_block = {}
    for p in design.PARTS:
        by_block.setdefault(p.block, []).append(p)
    flag_parts = [design.Part(f'#FLG0{i + 1}', 'power:PWR_FLAG', 'PWR_FLAG', '',
                              {'1': n}, block='misc')
                  for i, n in enumerate(design.PWR_FLAGS)]
    by_block['misc'].extend(flag_parts)

    def sym_for(lib_id):
        if lib_id not in lib_syms:
            lib_syms[lib_id] = load_symbol(lib_id)
        return lib_syms[lib_id]

    # Bloklari raf (shelf) yontemiyle sayfaya yerlestir
    page_w, x0, y0 = 560.0, 25.4, 30.48
    shelf_x, shelf_y, shelf_h = x0, y0, 0.0
    max_block_w = 250.0
    placed = []
    for blk in BLOCK_ORDER:
        parts = by_block.get(blk, [])
        rows, row, row_w = [], [], 0.0
        for p in parts:
            cw, ch = cell_size(p, sym_for(p.sym))
            if row and row_w + cw > max_block_w:
                rows.append(row)
                row, row_w = [], 0.0
            row.append((p, cw, ch))
            row_w += cw
        if row:
            rows.append(row)
        bw = max(sum(c[1] for c in r) for r in rows)
        bh = sum(max(c[2] for c in r) for r in rows) + 10.16
        if shelf_x + bw > page_w:
            shelf_x, shelf_y, shelf_h = x0, shelf_y + shelf_h + 10.16, 0.0
        bx, by = shelf_x, shelf_y
        sch.texts.append(Text(text=BLOCK_TITLES[blk],
                              position=Position(X=fx(snap(bx)), Y=fx(snap(by)), angle=0),
                              effects=Effects(font=Font(width=2.0, height=2.0, bold=True),
                                              justify=Justify(horizontally='left')),
                              uuid=str(uuid.uuid4())))
        cy = by + 7.62
        for r in rows:
            rh = max(c[2] for c in r)
            cx = bx
            for p, cw, ch in r:
                placed.append((p, cx, cy, cw, rh))
                cx += cw
            cy += rh
        shelf_x += bw + 12.7
        shelf_h = max(shelf_h, bh)

    for p, cx, cy, cw, ch in placed:
        sym = sym_for(p.sym)
        pins = pins_of(sym)
        xs = [q.position.X for q in pins] or [0]
        ys = [q.position.Y for q in pins] or [0]
        ext = side_extents(p, sym)
        right_free = ext[180] == 0
        sx = snap(cx + ext[0] + 3.81 - min(xs))
        sy = snap(cy + ext[90] + 2.54 + max(ys))

        top = sy - max(ys)
        bot = sy - min(ys)
        is_flag = p.ref.startswith('#')
        if right_free:
            rx = sx + max(xs + [2.54]) + 2.54
            ref_pos, val_pos = (rx, sy - 1.27), (rx, sy + 1.27)
        else:
            ref_pos, val_pos = (sx, top - 3.81), (sx, bot + 3.81)
        props = [
            prop('Reference', p.ref, *ref_pos, hide=is_flag, left=right_free),
            prop('Value', p.value, *val_pos, left=right_free),
            prop('Footprint', p.fp, sx, sy, hide=True),
            prop('Datasheet', '', sx, sy, hide=True),
        ]
        if p.mpn:
            props.append(prop('MPN', p.mpn, sx, sy, hide=True))
        if p.note:
            props.append(prop('Not', p.note, sx, sy, hide=True))
        for i, pr in enumerate(props):
            pr.id = i

        inst = SchematicSymbol(
            libraryNickname=sym.libraryNickname, entryName=sym.entryName,
            position=Position(X=fx(sx), Y=fx(sy), angle=0), unit=1,
            inBom=not is_flag, onBoard=not is_flag,
            uuid=str(uuid.uuid4()), properties=props,
            pins={q.number: str(uuid.uuid4()) for q in pins})
        sch.schematicSymbols.append(inst)
        sch.symbolInstances.append(SymbolInstance(
            path=f'/{inst.uuid}', reference=p.ref, unit=1, value=p.value,
            footprint=p.fp))

        for q in pins:
            px, py = fx(sx + q.position.X), fx(sy - q.position.Y)
            net = p.pins.get(q.number, '')
            if net:
                ang = (q.position.angle + 180) % 360
                sch.labels.append(LocalLabel(
                    text=net, position=Position(X=px, Y=py, angle=ang),
                    effects=label_effects(ang), uuid=str(uuid.uuid4())))
            elif q.electricalType != 'no_connect':
                sch.noConnects.append(NoConnect(position=Position(X=px, Y=py),
                                                uuid=str(uuid.uuid4())))

    sch.libSymbols = list(lib_syms.values())
    sch.sheetInstances = [HierarchicalSheetInstance(instancePath='/', page='1')]
    out = os.path.join(PROJ, 'avtech-agv-carrier.kicad_sch')
    sch.to_file(out)
    return out


if __name__ == '__main__':
    print(build())
