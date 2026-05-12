import numpy as np

def analyze_pcd_content():
    pcd_path = '../data/Town01.pcd'
    points   = []

    with open(pcd_path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('DATA'):
                break
            parts = line.split()
            if parts:
                header[parts[0]] = parts[1:]

        fields     = header.get('FIELDS', [])
        sizes      = [int(s) for s in header.get('SIZE', [])]
        count      = int(header.get('POINTS', ['0'])[0])
        point_step = sum(sizes)
        raw        = f.read(count * point_step)

        xi      = fields.index('x') if 'x' in fields else 0
        yi      = fields.index('y') if 'y' in fields else 1
        zi      = fields.index('z') if 'z' in fields else 2
        offsets = [sum(sizes[:i]) for i in range(len(sizes))]

        # Sample every 1000th point
        step = 1000
        for i in range(0, count, step):
            base  = i * point_step
            chunk = raw[base: base + point_step]
            if len(chunk) < point_step:
                break
            x = np.frombuffer(chunk[offsets[xi]: offsets[xi]+4], dtype=np.float32)[0]
            y = np.frombuffer(chunk[offsets[yi]: offsets[yi]+4], dtype=np.float32)[0]
            z = np.frombuffer(chunk[offsets[zi]: offsets[zi]+4], dtype=np.float32)[0]
            points.append((x, y, z))

    arr = np.array(points)
    print(f"Sampled {len(arr)} points\n")

    # Z distribution: tells us what heights exist
    print("=== Z distribution (what heights exist) ===")
    z_bins = np.arange(arr[:,2].min(), arr[:,2].max(), 1.0)
    hist, edges = np.histogram(arr[:,2], bins=50)
    for i in range(len(hist)):
        if hist[i] > 0:
            bar = '#' * min(50, hist[i] // 5)
            print(f"  z={edges[i]:7.2f} to {edges[i+1]:7.2f}  "
                  f"count={hist[i]:6d}  {bar}")

    # XY distribution: where are the points
    print(f"\n=== XY coverage ===")
    print(f"  X: {arr[:,0].min():.2f} to {arr[:,0].max():.2f}")
    print(f"  Y: {arr[:,1].min():.2f} to {arr[:,1].max():.2f}")

    # Points near a known road location (x=144, y=59 from CARLA)
    print(f"\n=== Points near known road location (144, 59) ===")
    near = arr[
        (np.abs(arr[:,0] - 144) < 50) &
        (np.abs(arr[:,1] - 59)  < 50)
    ]
    print(f"  Found {len(near)} points within 50m of (144, 59)")
    if len(near) > 0:
        print(f"  Z range of these points: {near[:,2].min():.3f} to {near[:,2].max():.3f}")
        print(f"  Sample:")
        for p in near[:5]:
            print(f"    x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}")

if __name__ == '__main__':
    analyze_pcd_content()