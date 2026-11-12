#!/usr/bin/env python3
"""Generate the `woollett_pool` SDF wrapper (visual = GLB, collision = boxes).

The visible geometry is a single GLB built by `build_woollett_glb.py`
(Blender headless). This script only emits the thin SDF that references the
GLB for the visual and adds cheap primitive-box collisions for physics.

    blender --background --python build_woollett_glb.py   # build the mesh
    python3 gen_woollett_pool.py                          # write SDF + config

Pool: 50 m (X) x 22.86 m (Y) x 2.1336 m (7 ft). Keep these in sync with the
PoolSpec in build_woollett_glb.py.
"""
import os
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.normpath(os.path.join(HERE, ".."))


@dataclass
class PoolSpec:
    length: float = 50.0
    width: float = 22.86
    depth: float = 2.1336
    wall_thickness: float = 0.2


def box_collision(name, x, y, z, sx, sy, sz):
    return f"""
      <collision name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>"""


def build_sdf(spec: PoolSpec):
    L, W, D = spec.length, spec.width, spec.depth
    t = spec.wall_thickness
    hl, hw = L / 2.0, W / 2.0
    cols = []
    cols.append(box_collision("floor_c", 0, 0, -D - 0.05, L, W, 0.1))
    for n, x, y, z, sx, sy, sz in [
        ("wall_xpos_c",  hl + t / 2, 0, -D / 2, t, W + 2 * t, D),
        ("wall_xneg_c", -hl - t / 2, 0, -D / 2, t, W + 2 * t, D),
        ("wall_ypos_c", 0,  hw + t / 2, -D / 2, L, t, D),
        ("wall_yneg_c", 0, -hw - t / 2, -D / 2, L, t, D),
    ]:
        cols.append(box_collision(n, x, y, z, sx, sy, sz))
    collisions = "\n".join(cols)

    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <model name="woollett_pool">
    <static>true</static>
    <link name="link">

      <!-- Visual: single GLB (built by build_woollett_glb.py via Blender) -->
      <visual name="pool">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://woollett_pool/meshes/woollett_pool.glb</uri></mesh>
        </geometry>
      </visual>

      <!-- Collisions: cheap primitive boxes (floor + 4 walls) -->
{collisions}
    </link>
  </model>
</sdf>
"""


def make_config(path):
    with open(path, "w") as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>Woollett Aquatics Centre Pool</name>
  <version>3.0.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <author><name>Beaverworks</name></author>
  <description>
    Woollett competition pool: 50 m x 22.86 m x 2.1336 m (7 ft). Visual is a
    single GLB (deep pool-blue floor, pale tile walls, 16 x 7 grid of 2.8 m
    navy squares) built by scripts/build_woollett_glb.py; collisions are
    primitive boxes from scripts/gen_woollett_pool.py.
  </description>
</model>
""")


def main():
    spec = PoolSpec()
    os.makedirs(MODEL_DIR, exist_ok=True)
    with open(os.path.join(MODEL_DIR, "model.sdf"), "w") as f:
        f.write(build_sdf(spec))
    make_config(os.path.join(MODEL_DIR, "model.config"))
    glb = os.path.join(MODEL_DIR, "meshes", "woollett_pool.glb")
    print(f"Wrote model.sdf + model.config in {MODEL_DIR}")
    print(f"  GLB present: {os.path.exists(glb)}  ({glb})")
    if not os.path.exists(glb):
        print("  !! run: blender --background --python build_woollett_glb.py")


if __name__ == "__main__":
    main()
