"""Sematik <-> design.py baglanti paritesi.

KiCad'in sematikten urettigi netlist (kicad-cli sch export netlist) ile
design.py'deki baglantilarin birebir ayni oldugunu dogrular.

    python3 check_parity.py <netlist.xml>
"""

import sys
import xml.etree.ElementTree as ET

import design


def kicad_nets(path):
    root = ET.parse(path).getroot()
    out = {}
    for net in root.iter('net'):
        name = net.get('name').lstrip('/')
        if name.startswith('unconnected-') or name.startswith('Net-('):
            # etiketsiz net = kopuk baglanti olabilir; ayrica raporla
            nodes = {(n.get('ref'), n.get('pin')) for n in net.iter('node')}
            if name.startswith('Net-('):
                out[name] = nodes
            continue
        nodes = {(n.get('ref'), n.get('pin')) for n in net.iter('node')
                 if not n.get('ref').startswith('#')}
        out[name] = nodes
    return out


def main(path):
    want = {n: set(c) for n, c in design.nets().items()}
    got = kicad_nets(path)
    errors = 0
    for n in sorted(set(want) | set(got)):
        a, b = want.get(n, set()), got.get(n, set())
        if a != b:
            errors += 1
            print(f'FARK {n}: eksik={sorted(a - b)} fazla={sorted(b - a)}')
    print(f'{len(want)} net kontrol edildi, {errors} fark')
    return errors


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1]) else 0)
