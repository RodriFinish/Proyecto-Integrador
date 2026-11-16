"""Corre la misión con el USB conectado y graba cada línea con su tiempo.

Conexión: VIN DESCONECTADA (si no, la corriente vuelve por el USB), batería en VM,
USB enchufado. Sostener el cable en alto mientras gira.

Uso: python scripts/mision_usb.py segundos archivo_salida
"""
import sys
import time

import serial

PORT = '/dev/cu.usbserial-A5069RR4'
secs = float(sys.argv[1])
out = sys.argv[2]

s = serial.Serial(PORT, 9600, timeout=0.05)
log = []  # (segundos desde que se abrió el puerto, línea)
t0 = time.time()
pend = b''
sent = started = False
tm = tr = None


def pump():
    global pend
    pend += s.read(512)
    *lines, pend = pend.split(b'\n')
    for l in lines:
        log.append((time.time() - t0, l.decode('latin1').strip()))


while time.time() - t0 < 25 + secs:
    pump()
    txt = '\n'.join(l for _, l in log[-30:])
    if not sent and 'cualquier tecla' in txt:
        time.sleep(0.2)
        s.write(b'x')
        sent = True
    elif sent and not started and 'm = arrancar' in txt:
        time.sleep(0.5)
        s.write(b'm')
        started = True
        tm = time.time()
    elif started and tr is None and time.time() - tm > secs:
        s.write(b'r')
        tr = time.time()
    elif tr and time.time() - tr > 4:
        break
s.close()

with open(out, 'w') as f:
    for t, l in log:
        f.write(f'{t:8.3f}\t{l}\n')
print('lineas', len(log), '| mision arrancada:', started)
print('\n'.join(l for _, l in log if not l.isdigit())[-3000:])
