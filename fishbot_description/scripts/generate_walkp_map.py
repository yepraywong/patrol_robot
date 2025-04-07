#!/usr/bin/env python3
"""
Script to generate a walkability index map PNG for an enclosed hallway simulation.
Created by: yepraywong
Date: 2025-04-06
Modified: Grid-based approach for discrete walkability indices without legend
"""

from PIL import Image, ImageDraw
import numpy as np
import os
import argparse

def create_walkability_index_map(output_path, width_meters=20.0, height_meters=8.0, resolution=300):
    """
    Create a walkability index map where:
    - Light green areas (value 1): Areas that can be walked on
    - Red areas (value 0): Slippery areas that cannot be walked on
    
    Parameters:
    - output_path: Path to save the generated PNG file
    - width_meters: Width of the hallway in meters (default: 10.0)
    - height_meters: Height of the hallway in meters (default: 5.0)
    - resolution: Pixels per meter (default: 100)
    """
    # Calculate dimensions in pixels
    width_px = int(width_meters * resolution)
    height_px = int(height_meters * resolution)
    
    # Calculate grid dimensions
    grid_size = resolution  # 1 grid cell = 1 meter
    grid_width = int(width_meters)
    grid_height = int(height_meters)
    
    # Create a walkability matrix (1: walkable, 0: non-walkable)
    # Initialize all cells as walkable (1)
    walkability_matrix = np.ones((grid_height, grid_width), dtype=np.int8)
    
    # Define non-walkable areas (index 0) in the grid
    # Non-walkable area 1 - near top right
    slip1_x = 7  # grid cell x-coordinate (from left)
    slip1_y = 1  # grid cell y-coordinate (from top)
    walkability_matrix[slip1_y, slip1_x] = 0
    
    # Non-walkable area 2 - near center
    slip2_x = 4  # grid cell x-coordinate
    slip2_y = 2  # grid cell y-coordinate
    walkability_matrix[slip2_y, slip2_x] = 0
    
    # Non-walkable area 3 - near bottom left
    slip3_x = 2  # grid cell x-coordinate
    slip3_y = 3  # grid cell y-coordinate
    walkability_matrix[slip3_y, slip3_x] = 0
    
    # Create a new image
    walkability_map = Image.new('RGB', (width_px, height_px), color=(255, 255, 255))
    draw = ImageDraw.Draw(walkability_map)
    
    # Draw the grid based on walkability matrix
    for y in range(grid_height):
        for x in range(grid_width):
            # Calculate pixel coordinates
            left = x * grid_size
            top = y * grid_size
            right = left + grid_size
            bottom = top + grid_size
            
            # Fill the cell based on walkability index
            if walkability_matrix[y, x] == 1:
                # Walkable area - light green
                fill_color = (200, 255, 200)
            else:
                # Non-walkable area - red
                fill_color = (255, 0, 0)
            
            # Draw the grid cell
            draw.rectangle([left, top, right, bottom], fill=fill_color, outline=(180, 230, 180))
    
    # Save the image
    walkability_map.save(output_path)
    print(f"Walkability index map has been created successfully at: {output_path}")
    
    # Display image dimensions and information
    print(f"Image dimensions: {width_px}x{height_px} pixels")
    print(f"Physical dimensions: {width_meters}x{height_meters} meters")
    print(f"Resolution: {resolution} pixels per meter")
    print(f"Grid dimensions: {grid_width}x{grid_height} cells")
    print("Map contains:")
    print("- Light green areas (value 1): Areas that can be walked on")
    print("- Red areas (value 0): Slippery areas that cannot be walked on")
    print("- 3 non-walkable grid cells")
    
    # Save walkability matrix for other applications if needed
    np.save(output_path.replace('.png', '_matrix.npy'), walkability_matrix)
    print(f"Walkability matrix has been saved as: {output_path.replace('.png', '_matrix.npy')}")

def main():
    parser = argparse.ArgumentParser(description='Generate walkability index map for enclosed hallway simulation')
    parser.add_argument('--output', '-o', type=str, default='walkability_map.png',
                        help='Output file path (default: walkability_index_map.png)')
    parser.add_argument('--width', '-w', type=float, default=20.0,
                        help='Width of the hallway in meters (default: 20.0)')
    parser.add_argument('--height', '-ht', type=float, default=8.0,
                        help='Height of the hallway in meters (default: 8.0)')
    parser.add_argument('--resolution', '-r', type=int, default=100,
                        help='Resolution in pixels per meter (default: 100)')
    
    args = parser.parse_args()
    
    create_walkability_index_map(args.output, args.width, args.height, args.resolution)

if __name__ == "__main__":
    main()