import serial, time, sys
s = serial.Serial('COM9', 115200, timeout=1)
print('opened COM9 @ 115200', flush=True)
start = time.time()
count = 0
while time.time() - start < 90:
    d = s.read(256)
    if d:
        count += len(d)
        try:
            sys.stdout.write(d.decode('utf-8', errors='replace'))
            sys.stdout.flush()
        except Exception as e:
            print(f'decode err: {e}', flush=True)
print(f'\n--- {count} bytes received in 150s ---', flush=True)
s.close()
