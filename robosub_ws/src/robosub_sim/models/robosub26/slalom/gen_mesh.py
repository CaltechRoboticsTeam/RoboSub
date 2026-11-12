from pathlib import Path

import bpy
import numpy as np

file_name = "mesh"
save_dir = Path("./")
save_dir.mkdir(parents=True, exist_ok=True)

pole_thickness = 0.05
pole_length = 0.9
channel_width = 1.5

# Adjust these to change the degree of winding
y_offset = 2.0
x_offsets = [0, -0.5, 0.5]

# Delete default objects
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False, confirm=False)

circleVertices = 32
add_pole = lambda location: bpy.ops.mesh.primitive_cylinder_add(
    radius=pole_thickness / 2,
    depth=pole_length,
    location=location,
    rotation=(np.pi, 0, 0),
    vertices=circleVertices,
)

red_mat = bpy.data.materials.new(name="RedMaterial")
red_mat.diffuse_color = (1, 0, 0, 1)  # RGBA for red

for i, x_offset in enumerate(x_offsets):
    y_pos = i * y_offset
    add_pole((x_offset - channel_width, y_pos, -pole_length))
    add_pole((x_offset, y_pos, -pole_length))
    obj = bpy.context.object
    obj.data.materials.append(red_mat)
    add_pole((x_offset + channel_width, y_pos, -pole_length))

model_path = save_dir / f"{file_name}.dae"
bpy.ops.wm.collada_export(filepath=str(model_path))
