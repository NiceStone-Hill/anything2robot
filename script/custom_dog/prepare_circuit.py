"""Copy the cactus concept circuit and explicitly extend it to five motor channels.

This preserves the Uno/switch/supply topology, not physical motor compatibility.
No motor voltage/protocol or current capacity is certified by this derivative.
"""
import copy
import csv
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
source = ROOT / 'data/electronics/cactus_tinkercad'
out = ROOT / 'data/custom_dog/circuit'
out.mkdir(exist_ok=True)
brd = ET.parse(next(source.glob('*.brd')))
sch = ET.parse(next(source.glob('*.sch')))
elements = brd.find('.//board/elements')
parts = sch.find('.//schematic/parts')
signals = brd.find('.//board/signals')
nets = sch.find('.//sheets/sheet/nets')
servo = next(p for p in parts if p.get('name') == 'SERVO1')
element = next(e for e in elements if e.get('name') == 'SERVO1')
for ref, digital, x in [('SERVO4', 'D5', '175'), ('SERVO5', 'D6', '215')]:
    p = copy.deepcopy(servo); p.set('name', ref); parts.append(p)
    e = copy.deepcopy(element); e.set('name', ref); e.set('x', x); elements.append(e)
    for signal in signals:
        for contact in list(signal.findall('contactref')):
            if contact.get('element') == 'SERVO1' and contact.get('pad') in ('1', '2'):
                new = copy.deepcopy(contact); new.set('element', ref); signal.append(new)
    # Exact original logical names for servo pins come from the source library.
    original_signal = next(n for n in nets if n.get('name') == 'U1_D10')
    original_refs = original_signal.findall('.//pinref')
    original_servo_pin = next(p for p in original_refs if p.get('part') == 'SERVO1')
    original_uno_pin = next(p for p in original_refs if p.get('part') == 'U1')
    signal = ET.SubElement(signals, 'signal', name=f'U1_{digital}')
    ET.SubElement(signal, 'contactref', element='U1', pad=digital)
    ET.SubElement(signal, 'contactref', element=ref, pad='3')
    net = ET.SubElement(nets, 'net', name=f'U1_{digital}', **{'class': '0'})
    segment = ET.SubElement(net, 'segment')
    for original, part, pin in [(original_servo_pin, ref, original_servo_pin.get('pin')),
                                 (original_uno_pin, 'U1', digital)]:
        new = copy.deepcopy(original); new.set('part', part); new.set('pin', pin); segment.append(new)
# Copy all supply pinrefs by identifying library pin -> board pad (not guessing names).
for ref in ['SERVO4', 'SERVO5']:
    for net in nets:
        if net.get('name') in ('P1-', 'SERVO2_PWR'):
            segment = net.find('segment')
            for original in net.findall('.//pinref'):
                if original.get('part') == 'SERVO1' and not any(p.get('part') == ref for p in net.findall('.//pinref')):
                    new = copy.deepcopy(original); new.set('part', ref); segment.append(new)
for tree, filename in [(brd, 'dog_five_channel.brd'), (sch, 'dog_five_channel.sch')]:
    ET.indent(tree); tree.write(out / filename, encoding='utf-8', xml_declaration=True)
with (source / 'bom.csv').open(encoding='utf-8-sig', newline='') as f:
    reader = csv.DictReader(f); fields = reader.fieldnames; rows = list(reader)
for row in rows:
    if 'SERVO1' in row['Name']:
        row['Name'] = 'SERVO1, SERVO2, SERVO3, SERVO4, SERVO5'; row['Quantity'] = '5'
with (out / 'bom.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
(out / 'adaptation.json').write_text(json.dumps({
    'source': str(source), 'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.suffix in ('.sch','.brd','.csv')},
    'changes': ['Add SERVO4/D5 and SERVO5/D6; retain original three channels, common ground and switched supply',
                'SCH/BRD logical connectivity and BOM updated together; source files unchanged',
                'Derived files are connectivity fixtures, not a routed PCB or a Tinkercad re-export'],
    'compatibility': 'Conceptual three-wire interfaces only; motor protocol, voltage and current require redesign/verification'
}, indent=2))
print(out)
