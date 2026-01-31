import serial, time
import matplotlib.pyplot as plt

PORT = "/dev/tty.usbmodem101"   # macOS example
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
xs, ys = [], []
t0 = None

fig = plt.figure()
line, = plt.plot([], [])

while True:
    s = ser.readline().decode(errors="ignore").strip()
    if not s:
        continue
    try:
        t, v = s.split(",")
        t = float(t); v = float(v)
    except:
        continue

    xs.append(t)
    ys.append(v)

    # keep last N seconds / samples
    if len(xs) > 2000:
        xs = xs[-2000:]
        ys = ys[-2000:]

    line.set_data(xs, ys)
    plt.xlim(xs[0], xs[-1] if xs[-1] > xs[0] else xs[0] + 1)
    plt.ylim(min(ys) - 0.1, max(ys) + 0.1)

    plt.pause(0.01)
