"""Uretim ciktilari: Gerber + delik (zip), dizgi pozisyon dosyasi, BOM,
sematik PDF, kart onizlemeleri.

    python3 export.py
"""

import csv
import os
import shutil
import subprocess
import zipfile

import design

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(HERE, '..', 'avtech-agv-carrier'))
OUT = os.path.normpath(os.path.join(HERE, '..', 'output'))
PCB = os.path.join(PROJ, 'avtech-agv-carrier.kicad_pcb')
SCH = os.path.join(PROJ, 'avtech-agv-carrier.kicad_sch')

LAYERS = 'F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts'


def cli(*args):
    env = dict(os.environ, KICAD7_FOOTPRINT_DIR='/usr/share/kicad/footprints',
               KICAD7_SYMBOL_DIR='/usr/share/kicad/symbols')
    r = subprocess.run(['kicad-cli', *args], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f'kicad-cli {" ".join(args)}\n{r.stdout}\n{r.stderr}')


def gerbers():
    gdir = os.path.join(OUT, 'gerber')
    shutil.rmtree(gdir, ignore_errors=True)
    os.makedirs(gdir)
    cli('pcb', 'export', 'gerbers', '-o', gdir + '/', '--layers', LAYERS,
        '--subtract-soldermask', PCB)
    cli('pcb', 'export', 'drill', '-o', gdir + '/', '--format', 'excellon',
        '--excellon-separate-th', '--generate-map', '--map-format', 'gerberx2', PCB)
    zpath = os.path.join(OUT, 'avtech-agv-carrier-revA-gerber.zip')
    with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(gdir)):
            z.write(os.path.join(gdir, f), f)
    return zpath, sorted(os.listdir(gdir))


def positions():
    path = os.path.join(OUT, 'avtech-agv-carrier-revA-pos.csv')
    cli('pcb', 'export', 'pos', '-o', path, '--format', 'csv', '--units', 'mm',
        '--side', 'both', '--smd-only', PCB)
    return path


def bom():
    groups = {}
    for p in design.PARTS:
        if p.ref.startswith(('H', 'TP')):
            continue
        key = (p.value, p.fp, p.mpn)
        groups.setdefault(key, []).append(p)

    def refkey(r):
        head = ''.join(c for c in r if c.isalpha())
        num = ''.join(c for c in r if c.isdigit())
        return head, int(num or 0)

    rows = []
    for (value, fp, mpn), parts in groups.items():
        refs = sorted((p.ref for p in parts), key=refkey)
        notes = sorted({p.note for p in parts if p.note})
        rows.append({'Adet': len(parts), 'Referanslar': ' '.join(refs), 'Deger': value,
                     'Footprint': fp.split(':')[1], 'Parca': mpn, 'Not': '; '.join(notes)})
    rows.sort(key=lambda r: refkey(r['Referanslar'].split()[0]))
    path = os.path.join(OUT, 'avtech-agv-carrier-revA-bom.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return path, rows


def schematic_pdf():
    path = os.path.join(OUT, 'avtech-agv-carrier-revA-schematic.pdf')
    cli('sch', 'export', 'pdf', '-o', path, SCH)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    z, files = gerbers()
    print('gerber:', z, len(files), 'dosya')
    print('pos:', positions())
    b, rows = bom()
    print('bom:', b, len(rows), 'satir,', sum(r['Adet'] for r in rows), 'parca')
    print('pdf:', schematic_pdf())


if __name__ == '__main__':
    main()
