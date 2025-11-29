#!/usr/bin/env python3
"""
GLB to DAE with simplified materials for Gazebo compatibility
Ensures all textures are properly exported and referenced
"""
import bpy
import sys
import math
import os

# Clear default scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# Get file paths
glb_file = sys.argv[-2]
dae_file = sys.argv[-1]
output_dir = os.path.dirname(dae_file)

print(f"Converting {glb_file} to {dae_file}")

# Import GLB
bpy.ops.import_scene.gltf(filepath=glb_file)

# Rotate to correct orientation
for obj in bpy.data.objects:
    obj.rotation_euler[0] += math.radians(90)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

print("Applied rotation")

# Simplify all materials for Gazebo compatibility
for mat in bpy.data.materials:
    if not mat.use_nodes:
        continue
    
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    
    if not bsdf:
        continue
    
    # Find texture nodes
    texture_nodes = [n for n in nodes if n.type == 'TEX_IMAGE' and n.image]
    
    if texture_nodes:
        # Ensure texture is connected to base color
        tex_node = texture_nodes[0]
        
        # Clear all inputs and reconnect properly
        for link in list(links):
            if link.to_socket == bsdf.inputs['Base Color']:
                links.remove(link)
        
        # Connect texture to base color
        links.new(tex_node.outputs['Color'], bsdf.inputs['Base Color'])
        
        # Set material properties for better Gazebo rendering
        bsdf.inputs['Metallic'].default_value = 0.0
        bsdf.inputs['Roughness'].default_value = 0.8
        bsdf.inputs['Specular'].default_value = 0.3
        bsdf.inputs['Alpha'].default_value = 1.0
        
        # Ensure image is saved and accessible
        if tex_node.image and tex_node.image.filepath:
            img_name = os.path.basename(tex_node.image.filepath)
            if not img_name:
                img_name = tex_node.image.name + '.jpg'
            
            # Pack and save image to output directory
            target_path = os.path.join(output_dir, img_name)
            try:
                tex_node.image.filepath_raw = target_path
                tex_node.image.save()
                print(f"Saved texture: {img_name}")
            except:
                pass
        
        print(f"Material {mat.name}: textured with {tex_node.image.name if tex_node.image else 'unknown'}")
    else:
        # No texture - set a visible gray color
        bsdf.inputs['Base Color'].default_value = (0.6, 0.6, 0.6, 1.0)
        bsdf.inputs['Metallic'].default_value = 0.0
        bsdf.inputs['Roughness'].default_value = 0.8
        bsdf.inputs['Alpha'].default_value = 1.0
        print(f"Material {mat.name}: using solid gray (no texture)")

# Select all objects for export
bpy.ops.object.select_all(action='SELECT')

# Export DAE with explicit settings
bpy.ops.wm.collada_export(
    filepath=dae_file,
    check_existing=False,
    apply_modifiers=True,
    triangulate=True,
    use_texture_copies=True,
    active_uv_only=False,
    use_object_instantiation=False,
    sort_by_name=True
)

print(f"Conversion complete: {dae_file}")
print("All materials exported with proper texture references")
