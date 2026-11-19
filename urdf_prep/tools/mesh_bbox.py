import glob, os, struct, sys

for f in sorted(glob.glob(sys.argv[1] + '/*.STL')):
    data = open(f, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + n * 50:
        print(os.path.basename(f), 'not binary STL?')
        continue
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for i in range(n):
        off = 84 + i * 50 + 12
        v = struct.unpack('<9f', data[off:off + 36])
        for k in range(3):
            for c in (v[k], v[k + 3], v[k + 6]):
                lo[k] = min(lo[k], c)
                hi[k] = max(hi[k], c)
    size = [h - l for h, l in zip(hi, lo)]
    print(f"{os.path.basename(f):26s} min {lo[0]:+.4f} {lo[1]:+.4f} {lo[2]:+.4f}  max {hi[0]:+.4f} {hi[1]:+.4f} {hi[2]:+.4f}  size {size[0]:.3f} {size[1]:.3f} {size[2]:.3f}  tris {n}")
