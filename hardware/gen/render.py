"""README icin kart gorseli: KiCad SVG ciktisindan ust ve alt yuz (alt yuz
aynalanmis) yan yana tek PNG.

    python3 render.py      # -> hardware/output/board_preview.png
Gerekenler: kicad-cli, rsvg-convert, ImageMagick (convert).
"""

import os
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.normpath(os.path.join(HERE, '..', 'avtech-agv-carrier', 'avtech-agv-carrier.kicad_pcb'))
OUT = os.path.normpath(os.path.join(HERE, '..', 'output', 'board_preview.png'))

SIDES = [
    ('top', 'F.Cu,F.SilkS,Edge.Cuts', []),
    ('bottom', 'B.Cu,B.SilkS,Edge.Cuts', ['--mirror']),
]


def main():
    env = dict(os.environ, KICAD7_FOOTPRINT_DIR='/usr/share/kicad/footprints')
    with tempfile.TemporaryDirectory() as tmp:
        pngs = []
        for name, layers, extra in SIDES:
            svg = os.path.join(tmp, f'{name}.svg')
            png = os.path.join(tmp, f'{name}.png')
            subprocess.run(['kicad-cli', 'pcb', 'export', 'svg', '-o', svg, '--layers', layers,
                            '--page-size-mode', '2', '--exclude-drawing-sheet', *extra, PCB],
                           check=True, capture_output=True, env=env)
            subprocess.run(['rsvg-convert', '-w', '900', '-b', '#ffffff', svg, '-o', png], check=True)
            pngs.append(png)
        subprocess.run(['convert', pngs[0], '-bordercolor', 'white', '-border', '20',
                        pngs[1], '-bordercolor', 'white', '-border', '20', '+append', OUT], check=True)
    print(OUT)


if __name__ == '__main__':
    main()
