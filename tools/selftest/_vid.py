
import glob, numpy as np
from PIL import Image

files = sorted(glob.glob("vid10/f_*.jpg"))
print("frames", len(files))

def heading(path):
    im = Image.open(path).convert("RGB")
    a = np.asarray(im).astype(np.int16)
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    red = (R > 110) & (R - G > 55) & (R - B > 55)
    dark = (R + G + B) < 210
    if red.sum() < 30 or dark.sum() < 3000:
        return None
    ys, xs = np.nonzero(red)
    ry, rx = ys.mean(), xs.mean()
    ys2, xs2 = np.nonzero(dark)
    cy, cx = ys2.mean(), xs2.mean()
    vx, vy = rx - cx, ry - cy
    ang = np.degrees(np.arctan2(vy, vx))
    return ang, red.sum(), dark.sum(), (rx, ry), (cx, cy)

out = []
for i, f in enumerate(files):
    h = heading(f)
    out.append((i / 10.0, h))

for t, h in out[::10]:
    if h:
        print("t=%5.1f  LED角度 %7.1f°  red=%4d dark=%5d" % (t, h[0], h[1], h[2]))
    else:
        print("t=%5.1f  ---" % t)
