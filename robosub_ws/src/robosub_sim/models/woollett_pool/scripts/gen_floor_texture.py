#!/usr/bin/env python3
"""Generate the repeating 2.8 m grid-cell texture for the Woollett pool floor.

Built on an EXACT integer tile lattice so every small tile is identical:
  cell = 2.8 m = 140 tiles (tile = 0.02 m), each tile TILE_PX wide, grout
  GROUT_PX wide. One cell is UV-tiled 16 x 7 across the floor (REPEAT), so the
  grid lines land on the 2.8 m box boundaries.

  - vertical lines  (straddle u edges) -> BLACK,  10 tiles wide, continuous
  - horizontal lines(straddle v edges) -> DARK BLUE, 10 tiles wide, but each
    blue segment STOPS GAP_TILES short of every black line (blue passes under
    the black and never touches it)
  - field is solid baby blue (no tiles/grout)

Output: ../models/woollett_pool/meshes/floor_tiles.png
"""
import os
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(
    HERE, "..", "meshes", "floor_tiles.png"))

N = 140            # tiles per 2.8 m cell  (tile = 0.02 m)
TILE_PX = 15       # pixels per tile  -> CELL_PX = 2100
GROUT_PX = 2       # grout width in pixels
HALF_BAND = 5      # line half-width in tiles -> 10 tiles wide
GAP_TILES = 3      # blue stops this many tiles short of each black line

CELL_PX = N * TILE_PX

FIELD = np.array([0.60, 0.82, 0.92])     # baby blue
DARKBLUE = np.array([0.03, 0.10, 0.42])  # horizontal lines
BLACK = np.array([0.02, 0.02, 0.03])     # vertical lines
GROUT = np.array([0.93, 0.95, 0.97])     # white grout


def main():
    idx = np.arange(CELL_PX)
    u_tile = idx / TILE_PX                       # tile coordinate 0..N
    du = np.minimum(u_tile, N - u_tile)          # tiles to nearest u-boundary
    dv = du.copy()                               # symmetric in v

    DU, DV = np.meshgrid(du, dv)                 # DU varies along x(cols)
    in_black = DU < HALF_BAND                                  # vertical band
    in_blue = (DV < HALF_BAND) & (DU >= HALF_BAND + GAP_TILES) # horiz + gap

    within = idx % TILE_PX
    grout1d = within < GROUT_PX
    GX, GY = np.meshgrid(grout1d, grout1d)
    is_grout = GX | GY

    img = np.ones((CELL_PX, CELL_PX, 3)) * FIELD
    img[in_blue] = np.where(is_grout[in_blue, None], GROUT, DARKBLUE)
    img[in_black] = np.where(is_grout[in_black, None], GROUT, BLACK)  # on top

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype("uint8")).save(OUT)
    print(f"wrote {OUT}  ({CELL_PX}x{CELL_PX}px, {N} tiles/cell, "
          f"line {2*HALF_BAND} tiles wide, gap {GAP_TILES} tiles)")


if __name__ == "__main__":
    main()
