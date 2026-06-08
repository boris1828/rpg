import bpy
import math

obj = bpy.data.objects.get("carrot")
if obj is None:
    raise RuntimeError("Object 'carrot' not found")

obj.rotation_mode = 'ZXY'

start_frame = 1
mid_frame   = 6
end_frame   = 12
z_offset    = 0.1

tilt_x = math.radians(-35.0)  # fixed backward tilt

start_location = obj.location.copy()
start_location.z = 0.0

# Frame 1
bpy.context.scene.frame_set(start_frame)
obj.location = start_location
obj.rotation_euler = (
    tilt_x,                 # fixed X tilt
    0.0,
    0.0
)
obj.keyframe_insert(data_path="location")
obj.keyframe_insert(data_path="rotation_euler")

# Frame 6
bpy.context.scene.frame_set(mid_frame)
obj.location.z = start_location.z + z_offset
obj.rotation_euler = (
    tilt_x,                 # SAME tilt
    0.0,
    math.pi / 4.0
)
obj.keyframe_insert(data_path="location")
obj.keyframe_insert(data_path="rotation_euler")

# Frame 12
bpy.context.scene.frame_set(end_frame)
obj.location = start_location
obj.rotation_euler = (
    tilt_x,                 # SAME tilt
    0.0,
    math.pi / 2.0
)
obj.keyframe_insert(data_path="location")
obj.keyframe_insert(data_path="rotation_euler")

bpy.context.scene.frame_start = start_frame
bpy.context.scene.frame_end   = end_frame

# Exporting data

import bpy
import os

obj = bpy.data.objects["carrot"]

start   = 1
end     = 12
out_dir = r"C:\Users\Workstation\rpg\assets\moving_carrot"

# Create directory if it doesn't exist
os.makedirs(out_dir, exist_ok=True)

for frame in range(start, end + 1):
    bpy.context.scene.frame_set(frame)

    bpy.ops.wm.obj_export(
        filepath=os.path.join(out_dir, f"carrot_{frame:02d}.obj"),
        export_selected_objects = True,
        apply_modifiers         = True,
        export_uv               = False,
        export_normals          = True,
        export_materials        = False,
        export_colors           = False,
        forward_axis='Y',
        up_axis='Z'
    )
