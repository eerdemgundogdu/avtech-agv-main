"""Tum karti bastan uretir: kutuphane, sematik, parite kontrolu, yerlesim,
yol cizimi, dokum, DRC ve uretim ciktilari.

    cd hardware/gen && python3 build.py

Gerekenler: KiCad 7 (pcbnew Python modulu + kicad-cli), kiutils, numpy, scipy.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault('KICAD7_FOOTPRINT_DIR', '/usr/share/kicad/footprints')
os.environ.setdefault('KICAD7_SYMBOL_DIR', '/usr/share/kicad/symbols')
sys.path.insert(0, HERE)

import check_parity  # noqa: E402
import export  # noqa: E402
import finish_pcb  # noqa: E402
import make_lib  # noqa: E402
import make_sch  # noqa: E402
import render  # noqa: E402
import route_pcb  # noqa: E402

OUT = os.path.normpath(os.path.join(HERE, '..', 'output'))


def main():
    os.makedirs(OUT, exist_ok=True)
    make_lib.main()
    sch = make_sch.build()
    net = os.path.join(OUT, 'netlist.xml')
    subprocess.run(['kicad-cli', 'sch', 'export', 'netlist', '--format', 'kicadxml',
                    '-o', net, sch], check=True, capture_output=True)
    if check_parity.main(net):
        sys.exit('sematik ile design.py uyusmuyor')
    os.remove(net)

    _, r = route_pcb.run(verbose=False)
    with open(os.path.join(OUT, 'routing_report.txt'), 'w') as f:
        f.write(f'{len(r.tracks)} iz parcasi, {len(r.vias)} via '
                f'({getattr(r, "stitch_count", 0)} tanesi GND dikis via)\n')
        f.write(f'basarisiz baglanti: {len(r.failed)}\n')
        for n in r.failed:
            f.write(f'  BASARISIZ {n}\n')
        f.write('baştan ince yonlendirilen dallar (net, ped, istenen genislik mm):\n')
        for n in r.necked:
            f.write(f'  {n[0]} -> {n[1]} (istenen {n[2]} mm, cizilen 0.3 mm)\n')
    counts, kinds = finish_pcb.main()
    export.main()
    render.main()
    if counts['unconnected'] or r.failed:
        sys.exit('bagli olmayan ped var')


if __name__ == '__main__':
    main()
