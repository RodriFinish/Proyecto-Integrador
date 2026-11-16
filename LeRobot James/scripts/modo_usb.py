"""Abre el puerto, deja el robot en MODO USB (manda una tecla en la cuenta regresiva
o en el aviso de datos sin leer) y graba las distancias durante N segundos.

Uso: python scripts/modo_usb.py [segundos] [archivo_salida]
"""
import statistics as st
import sys
import time

import serial

PORT = '/dev/cu.usbserial-A5069RR4'
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5
out = sys.argv[2] if len(sys.argv) > 2 else None

s = serial.Serial(PORT, 9600, timeout=0.1)
buf = b''
t0 = time.time()
sent_at = None
while time.time() - t0 < 15:
    buf += s.read(512)
    txt = buf.decode('latin1')
    if sent_at is None and 'cualquier tecla' in txt:
        time.sleep(0.2)
        s.write(b'x')
        sent_at = time.time()
    if sent_at and 'm = arrancar' in txt and time.time() - sent_at > 0.5:
        break
boot = buf.decode('latin1')

s.reset_input_buffer()
buf = b''
t1 = time.time()
while time.time() - t1 < secs:
    buf += s.read(512)
s.close()

vals = [int(l) for l in (x.strip() for x in buf.decode('latin1').splitlines()) if l.isdigit()]
print(boot)
print('MODO USB:', 'si' if 'MODO USB' in boot else 'NO (revisar)')
print('lecturas:', len(vals), 'en', secs, 's')
if vals:
    print('min', min(vals), 'max', max(vals), 'mediana', st.median(vals),
          'desvio %.1f' % st.pstdev(vals))
if out:
    with open(out, 'w') as f:
        f.write(boot + '\n--- LECTURAS ---\n' + '\n'.join(map(str, vals)) + '\n')
