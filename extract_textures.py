#!/usr/bin/env python3
"""
Extract all textures from GLB file and save them to the model directory
"""
import bpy
import sys
import os
import shutil

# Clear default scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# Get file paths
glb_file = sys.argv[-2]
output_dir = sys.argv[-1]

print(f"Extracting textures from {glb_file} to {output_dir}")

# Import GLB
bpy.ops.import_scene.gltf(filepath=glb_file)

# Create output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

texture_count = 0

# Extract all textures
for img in bpy.data.images:
    if img.type == 'IMAGE':
        # Get image name
        img_name = img.name
        
        # Ensure proper extension
        if not any(img_name.lower().endswith(ext) for ext in ['.png', '.jpg', '.jpeg', '.bmp', '.tga']):
            img_name += '.png'
        
        # Create output path
        output_path = os.path.join(output_dir, img_name)
        
        # Save image
        try:
            # Set file format based on extension
            old_format = img.file_format
            if img_name.lower().endswith('.png'):
                img.file_format = 'PNG'
            elif img_name.lower().endswith(('.jpg', '.jpeg')):
                img.file_format = 'JPEG'
            
            img.filepath_raw = output_path
            img.save()
            print(f"Saved texture: {img_name}")
            texture_count += 1
            
            # Restore format
            img.file_format = old_format
        except Exception as e:
            print(f"Error saving {img_name}: {e}")

print(f"\nTotal textures extracted: {texture_count}")
