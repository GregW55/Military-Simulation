"""
make_terrain.py: one-time data prep for the 3D terrain.

Outputs (in ./out):
  terrain.bin               heights in meters on a lat/lon grid (read by C++)
  tiles/L{level}_{kkk}.jpg  satellite picture per terrain chunk, one set per detail level
                            (kkk = chunkRow*CHUNKS + chunkCol, row 0 = north)
  preview.jpg               whole-box overview, to check the area looks right
  elevation_preview.png     grayscale heights, to check elevation looks right

Setup:  pip install numpy pillow scipy requests
Credit in your README: Sentinel-2 cloudless by EOX (Contains modified Copernicus
Sentinel data), terrain from AWS Terrain Tiles.
"""
import io, math, os, struct, time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import requests
from PIL import Image
from scipy.ndimage import map_coordinates

# ============================ SETTINGS ============================
LAT_MIN, LAT_MAX = 17.5, 31.5          # about 840 nm tall
LON_MIN, LON_MAX = 113.25, 128.75      # about 836 nm wide, centered on Taiwan

CHUNKS = 16                            # 16x16 terrain chunks (must match the C++ side)
CHUNK_CELLS = 128                      # mesh cells per chunk edge
GRID_W = GRID_H = CHUNKS * CHUNK_CELLS + 1     # 2049 vertices per side
ZOOM_DEM = 9                           # source elevation detail (~280 m per pixel)

LEVELS = [0, 1, 2, 3, 4]                           # start with [0]; then add 1, 2, 3, 4 one at a time
LEVEL_ZOOM = {0: 9, 1: 10, 2: 11, 3: 12, 4: 13}          # source imagery zoom per level
LEVEL_PX   = {0: 256, 1: 512, 2: 1024, 3: 2048, 4: 4096} # output picture size per chunk

# Expensive levels are only baked over the area you'll look at closely:
# (lat_min, lat_max, lon_min, lon_max). Levels not listed cover the whole box.
INSET = {4: (21.5, 26.5, 118.0, 122.8)}

JPEG_QUALITY = 90
SHADE_EXAG = 2.0                       # exaggerates relief in the shading only
WORKERS = 8                            # parallel downloads (lower to 4 if you get errors)
PREVIEW_PX = 2048 // CHUNKS            # per-chunk size in the overview image

# Open ocean is replaced with a clean depth-tinted color (the satellite ocean has stripes)
OCEAN_SHALLOW = np.array([52, 102, 128], np.float32)   # color in shallow water
OCEAN_DEEP    = np.array([6, 28, 58], np.float32)      # color in deep water
OCEAN_BLEND_START_M = 8      # shallower than this keeps the satellite picture
OCEAN_BLEND_FULL_M  = 40     # deeper than this uses the clean ocean color
OCEAN_DEEP_M = 1500          # depth where the color reaches "deep"

DEM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
IMG_URL = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2020_3857/default/g/{z}/{y}/{x}.jpg"

OUT_DIR, TILE_DIR, CACHE_DIR = "out", "out/tiles", "cache"
HEADERS = {"User-Agent": "military-sim-terrain-prep"}
# ==================================================================


# ---- Web Mercator math (only used here, never in C++) ----
def merc_x(lon, z):
    return (np.asarray(lon, dtype=np.float64) + 180.0) / 360.0 * 256 * 2 ** z

def merc_y(lat, z):
    s = np.sin(np.radians(np.asarray(lat, dtype=np.float64)))
    return (0.5 - np.log((1 + s) / (1 - s)) / (4 * np.pi)) * 256 * 2 ** z


# ---- Downloading, with an on-disk cache so reruns and crashes cost nothing ----
def get_tile(template, name, z, x, y):
    path = os.path.join(CACHE_DIR, f"{name}_{z}_{x}_{y}")
    if os.path.exists(path):
        with open(path, "rb") as f:
            return f.read()
    url = template.format(z=z, x=x, y=y)
    for attempt in range(4):
        try:
            r = requests.get(url, headers=HEADERS, timeout=30)
            r.raise_for_status()
            break
        except requests.RequestException as e:
            if attempt == 3:
                raise RuntimeError(f"Download failed: {url}\n  {e}")
            time.sleep(1.5 * (attempt + 1))
    os.makedirs(CACHE_DIR, exist_ok=True)
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(r.content)
    os.replace(tmp, path)
    return r.content


def decode_dem(blob):
    # Terrarium encoding: height_m = R*256 + G + B/256 - 32768
    a = np.asarray(Image.open(io.BytesIO(blob)).convert("RGBA").convert("RGB"), dtype=np.float32)
    return (a[:, :, 0] * 256 + a[:, :, 1] + a[:, :, 2] / 256 - 32768)[:, :, None]

def decode_img(blob):
    return np.asarray(Image.open(io.BytesIO(blob)).convert("RGBA").convert("RGB"), dtype=np.uint8)


def build_mosaic(template, name, z, lat_min, lat_max, lon_min, lon_max, decode):
    """Download every tile covering the box and stitch them. Returns (array, originX, originY)."""
    n = 2 ** z
    x0 = max(0, int(merc_x(lon_min, z) // 256)); x1 = min(n - 1, int(merc_x(lon_max, z) // 256))
    y0 = max(0, int(merc_y(lat_max, z) // 256)); y1 = min(n - 1, int(merc_y(lat_min, z) // 256))
    coords = [(tx, ty) for ty in range(y0, y1 + 1) for tx in range(x0, x1 + 1)]
    print(f"    {len(coords)} tiles at zoom {z}", flush=True)
    with ThreadPoolExecutor(WORKERS) as ex:
        blobs = list(ex.map(lambda c: get_tile(template, name, z, c[0], c[1]), coords))
    first = decode(blobs[0])
    out = np.zeros(((y1 - y0 + 1) * 256, (x1 - x0 + 1) * 256, first.shape[2]), first.dtype)
    for (tx, ty), blob in zip(coords, blobs):
        out[(ty - y0) * 256:(ty - y0 + 1) * 256, (tx - x0) * 256:(tx - x0 + 1) * 256] = decode(blob)
    return out, x0 * 256, y0 * 256


def sample(mosaic, ox, oy, z, lats, lons, band=512):
    """Read the mosaic at a regular lat/lon grid. Returns (len(lats), len(lons), channels)."""
    # -0.5 because Mercator pixel coordinates mark pixel edges, array indices mark centers
    px = merc_x(lons, z) - ox - 0.5
    py = merc_y(lats, z) - oy - 0.5
    # Crop to the window we actually need (small pad for interpolation)
    x0 = max(int(np.floor(px.min())) - 2, 0); x1 = min(int(np.ceil(px.max())) + 3, mosaic.shape[1])
    y0 = max(int(np.floor(py.min())) - 2, 0); y1 = min(int(np.ceil(py.max())) + 3, mosaic.shape[0])
    win = mosaic[y0:y1, x0:x1]
    out = np.empty((len(lats), len(lons), mosaic.shape[2]), np.float32)
    for r in range(0, len(lats), band):            # a band of rows at a time keeps memory small
        gy, gx = np.meshgrid(py[r:r + band] - y0, px - x0, indexing="ij")
        for c in range(mosaic.shape[2]):
            out[r:r + band, :, c] = map_coordinates(win[:, :, c], [gy, gx], order=1,
                                                    mode="nearest", output=np.float32)
    return out


# ---- Elevation ----
def build_elevation():
    lats = np.linspace(LAT_MAX, LAT_MIN, GRID_H)      # row 0 = north edge
    lons = np.linspace(LON_MIN, LON_MAX, GRID_W)
    mos, ox, oy = build_mosaic(DEM_URL, "dem", ZOOM_DEM, LAT_MIN, LAT_MAX, LON_MIN, LON_MAX, decode_dem)
    h_raw = sample(mos, ox, oy, ZOOM_DEM, lats, lons)[:, :, 0].astype(np.float32)
    return np.maximum(h_raw, 0), h_raw, lats          # terrain.bin gets the clamped version


def write_terrain_bin(h):
    with open(os.path.join(OUT_DIR, "terrain.bin"), "wb") as f:
        f.write(struct.pack("<ii4f", GRID_W, GRID_H, LAT_MIN, LAT_MAX, LON_MIN, LON_MAX))
        f.write(h.astype("<f4").tobytes())


def hillshade(h, lats):
    """Brightness factor per grid cell: 1.0 on flat ground, lower on slopes facing away from the NW sun."""
    dy = (LAT_MAX - LAT_MIN) / (GRID_H - 1) * 110540.0                              # meters per row
    dx = (LON_MAX - LON_MIN) / (GRID_W - 1) * 111320.0 * np.cos(np.radians(lats))   # meters per col, per row
    dzdn = -np.gradient(h, axis=0) / dy * SHADE_EXAG       # rows go south, so flip for "per north"
    dzde = np.gradient(h, axis=1) / dx[:, None] * SHADE_EXAG
    az, alt = math.radians(315), math.radians(45)
    lx, ly, lz = math.cos(alt) * math.sin(az), math.cos(alt) * math.cos(az), math.sin(alt)
    shade = (-dzde * lx - dzdn * ly + lz) / np.sqrt(dzde ** 2 + dzdn ** 2 + 1)
    return np.clip(shade / lz, 0, 1.6).astype(np.float32)


# ---- Imagery, one chunk at a time ----
def chunk_bounds(cx, cy):
    lon0 = LON_MIN + (LON_MAX - LON_MIN) * cx / CHUNKS
    lon1 = LON_MIN + (LON_MAX - LON_MIN) * (cx + 1) / CHUNKS
    lat1 = LAT_MAX - (LAT_MAX - LAT_MIN) * cy / CHUNKS            # north edge
    lat0 = LAT_MAX - (LAT_MAX - LAT_MIN) * (cy + 1) / CHUNKS      # south edge
    return lat0, lat1, lon0, lon1


def in_inset(level, cx, cy):
    if level not in INSET:
        return True
    la0, la1, lo0, lo1 = INSET[level]
    c_south, c_north, c_west, c_east = chunk_bounds(cx, cy)
    return c_north > la0 and c_south < la1 and c_east > lo0 and c_west < lo1


def build_chunk_image(level, cx, cy, shade, h_raw):
    lat0, lat1, lon0, lon1 = chunk_bounds(cx, cy)
    z, px = LEVEL_ZOOM[level], LEVEL_PX[level]
    # Pixel CENTERS, so the picture's edges land exactly on the chunk's edges
    lons = lon0 + (np.arange(px) + 0.5) / px * (lon1 - lon0)
    lats = lat1 - (np.arange(px) + 0.5) / px * (lat1 - lat0)

    mos, ox, oy = build_mosaic(IMG_URL, "img", z, lat0, lat1, lon0, lon1, decode_img)
    rgb = sample(mos, ox, oy, z, lats, lons)

    # Where each pixel falls in the elevation grid
    gi = (lons - LON_MIN) / (LON_MAX - LON_MIN) * (GRID_W - 1)
    gj = (LAT_MAX - lats) / (LAT_MAX - LAT_MIN) * (GRID_H - 1)
    gjj, gii = np.meshgrid(gj, gi, indexing="ij")

    # Replace striped open ocean with a clean color tinted by water depth
    depth = np.maximum(-map_coordinates(h_raw, [gjj, gii], order=1, mode="nearest"), 0)
    water = np.clip((depth - OCEAN_BLEND_START_M) / (OCEAN_BLEND_FULL_M - OCEAN_BLEND_START_M), 0, 1)[:, :, None]
    t = np.clip(np.sqrt(depth / OCEAN_DEEP_M), 0, 1)[:, :, None]
    ocean = OCEAN_SHALLOW * (1 - t) + OCEAN_DEEP * t
    rgb = rgb * (1 - water) + ocean * water

    # Multiply by the hillshade so relief shows
    s = map_coordinates(shade, [gjj, gii], order=1, mode="nearest")
    factor = np.clip(0.5 + 0.5 * s, 0.4, 1.3)[:, :, None]
    return np.clip(rgb * factor, 0, 255).astype(np.uint8)


def main():
    assert -85 < LAT_MIN < LAT_MAX < 85, f"Bad latitudes {LAT_MIN}, {LAT_MAX}"
    assert -180 < LON_MIN < LON_MAX < 180, f"Bad longitudes {LON_MIN}, {LON_MAX}"
    os.makedirs(TILE_DIR, exist_ok=True)
    print(f"Box: lat {LAT_MIN} to {LAT_MAX}, lon {LON_MIN} to {LON_MAX}, grid {GRID_W}x{GRID_H}")

    print("Elevation...", flush=True)
    h, h_raw, lats = build_elevation()
    write_terrain_bin(h)
    print(f"  max height {h.max():.0f} m (Yushan, ~3,950 m real; expect 3,300 to 3,900 on this grid), "
          f"{(h > 0).mean() * 100:.0f}% of cells are land")
    Image.fromarray((255 * h / max(h.max(), 1)).astype(np.uint8)).save(
        os.path.join(OUT_DIR, "elevation_preview.png"))
    shade = hillshade(h, lats)

    preview_done = False
    for level in LEVELS:
        # Inset-only levels cover part of the box, so they never write the overview
        preview_done = preview_done or (level in INSET)

        print(f"Imagery level {level} ({LEVEL_PX[level]}px per chunk, zoom {LEVEL_ZOOM[level]})", flush=True)
        keep = {}
        for cy in range(CHUNKS):
            for cx in range(CHUNKS):
                if not in_inset(level, cx, cy):
                    continue
                k = cy * CHUNKS + cx
                print(f"  chunk {k + 1}/{CHUNKS * CHUNKS}", flush=True)
                img = build_chunk_image(level, cx, cy, shade, h_raw)
                Image.fromarray(img).save(os.path.join(TILE_DIR, f"L{level}_{k:03d}.jpg"),
                                          quality=JPEG_QUALITY, optimize=True)
                if not preview_done:            # keep only a small copy for the overview
                    keep[(cx, cy)] = np.asarray(
                        Image.fromarray(img).resize((PREVIEW_PX, PREVIEW_PX), Image.LANCZOS))

        if not preview_done:                    # whole-box overview from the first full level
            big = np.zeros((CHUNKS * PREVIEW_PX, CHUNKS * PREVIEW_PX, 3), np.uint8)
            for (cx, cy), small in keep.items():
                big[cy * PREVIEW_PX:(cy + 1) * PREVIEW_PX,
                cx * PREVIEW_PX:(cx + 1) * PREVIEW_PX] = small
            Image.fromarray(big).save(os.path.join(OUT_DIR, "preview.jpg"), quality=90)
            preview_done = True
            print("  wrote out/preview.jpg: open it and check it looks right before continuing")
    print("Done.")


if __name__ == "__main__":
    main()