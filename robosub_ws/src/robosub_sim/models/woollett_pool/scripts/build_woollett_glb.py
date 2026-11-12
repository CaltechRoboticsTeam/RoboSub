"""Build the Woollett pool as a single GLB (run inside Blender headless).

    python3 gen_floor_texture.py                          # 1. cell texture
    blender --background --python build_woollett_glb.py   # 2. this -> GLB

Geometry (Blender Z-up; exported Z-up so gz shows it upright):
  50 m (X) x 22.86 m (Y) x 2.1336 m (7 ft) basin.

ONE textured material is used for the whole pool so the floor, the margins and
the walls are guaranteed the exact same baby-blue shade (a solid material vs a
texture would differ because gz sRGB-decodes images but not solid colours):
  - floor plane spans the full footprint; its UV is offset so the 16 x 7 grid
    of 2.8 m mosaic lines lands on the grid boundaries and the margins stay
    pure field (baby blue).
  - walls pin every UV to a field texel -> uniform baby blue.
"""
import bpy
import os

L, W, D = 50.0, 22.86, 2.1336
GRID = 2.8
NX, NY = 16, 7
WALL_T = 0.2

# first grid line position (grid centred in the pool)
GX0 = -NX * GRID / 2.0   # -22.4
GY0 = -NY * GRID / 2.0   # -9.8

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.normpath(os.path.join(HERE, "..", "meshes"))
TEX = os.path.join(MESH_DIR, "floor_tiles.png")
OUT = os.path.join(MESH_DIR, "woollett_pool.glb")

FIELD_UV = (0.5, 0.5)    # a pure-field texel (cell centre)


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for b in list(block):
            block.remove(b)


def make_tex_mat(name, png, emis):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes.get("Principled BSDF")
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(png)
    tex.extension = 'REPEAT'
    tex.interpolation = 'Linear'
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    for ec in ("Emission Color", "Emission"):
        if ec in bsdf.inputs:
            nt.links.new(tex.outputs["Color"], bsdf.inputs[ec])
            break
    if "Emission Strength" in bsdf.inputs:
        bsdf.inputs["Emission Strength"].default_value = emis
    if "Roughness" in bsdf.inputs:
        bsdf.inputs["Roughness"].default_value = 0.6
    return m


def _ensure_uv(me):
    if not me.uv_layers:
        me.uv_layers.new()
    return me.uv_layers[0].data


def add_grid_plane(name, center, sx, sy, mat):
    """Plane carrying the tiled grid texture (UV = grid mapping)."""
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=center)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (sx, sy, 1.0)
    bpy.ops.object.transform_apply(scale=True)
    me = ob.data
    uvl = _ensure_uv(me)
    for poly in me.polygons:
        for li in poly.loop_indices:
            co = me.vertices[me.loops[li].vertex_index].co
            uvl[li].uv = ((co.x - GX0) / GRID, (co.y - GY0) / GRID)
    ob.data.materials.append(mat)
    return ob


def add_field_plane(name, center, sx, sy, mat):
    """Plane that is pure field colour (UV pinned to a field texel)."""
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=center)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (sx, sy, 1.0)
    bpy.ops.object.transform_apply(scale=True)
    me = ob.data
    uvl = _ensure_uv(me)
    for li in range(len(uvl)):
        uvl[li].uv = FIELD_UV
    ob.data.materials.append(mat)
    return ob


def add_box(name, center, size, mat, const_uv):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=center)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (size[0], size[1], size[2])
    bpy.ops.object.transform_apply(scale=True)
    me = ob.data
    uvl = _ensure_uv(me)
    for li in range(len(uvl)):
        uvl[li].uv = const_uv
    ob.data.materials.append(mat)
    return ob


def main():
    clear_scene()
    mat = make_tex_mat("pool", TEX, 0.22)

    floor_top = -D
    # full-footprint base: pure field (lines never reach the pool edges)
    add_field_plane("floor", (0, 0, floor_top), L, W, mat)
    # tiled grid only over the 16x7 region (+ a half-line border so the outer
    # box borders render full), sitting just above the base
    border = 0.12
    add_grid_plane("grid", (0, 0, floor_top + 0.005),
                   NX * GRID + 2 * border, NY * GRID + 2 * border, mat)
    hl, hw = L / 2.0, W / 2.0
    for n, c, s in [
        ("wall_xpos", (hl + WALL_T / 2, 0, -D / 2), (WALL_T, W + 2 * WALL_T, D)),
        ("wall_xneg", (-hl - WALL_T / 2, 0, -D / 2), (WALL_T, W + 2 * WALL_T, D)),
        ("wall_ypos", (0, hw + WALL_T / 2, -D / 2), (L, WALL_T, D)),
        ("wall_yneg", (0, -hw - WALL_T / 2, -D / 2), (L, WALL_T, D)),
    ]:
        add_box(n, c, s, mat, FIELD_UV)

    os.makedirs(MESH_DIR, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=OUT,
        export_format='GLB',
        use_selection=False,
        export_apply=True,
        export_yup=False,          # keep Z-up; gz reads coords literally
    )
    print(f"[woollett] exported -> {OUT}")


if __name__ == "__main__":
    main()
