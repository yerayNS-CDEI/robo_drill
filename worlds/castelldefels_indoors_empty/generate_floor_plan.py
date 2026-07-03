#!/usr/bin/env python3
"""
Generate a 2D floor plan from Gazebo world file.
Creates SVG and PNG floor plans with measurements.
"""

import xml.etree.ElementTree as ET
import math
import os


def parse_pose(pose_str):
    """Parse pose string to [x, y, z, roll, pitch, yaw]."""
    if pose_str is None:
        return [0, 0, 0, 0, 0, 0]
    values = [float(x) for x in pose_str.split()]
    if len(values) == 6:
        return values
    return [0, 0, 0, 0, 0, 0]


def parse_box_size(size_str):
    """Parse box size string to [x, y, z]."""
    return [float(x) for x in size_str.split()]


def rotate_point(x, y, yaw):
    """Rotate a point around origin by yaw angle."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_rot = x * cos_yaw - y * sin_yaw
    y_rot = x * sin_yaw + y * cos_yaw
    return x_rot, y_rot


def get_rectangle_corners(size_x, size_y, pose):
    """Get the 4 corners of a rectangle in 2D (top-down view)."""
    x, y, z, roll, pitch, yaw = pose
    
    # Half dimensions
    hx = size_x / 2
    hy = size_y / 2
    
    # Four corners relative to center
    corners = [
        (-hx, -hy),
        (hx, -hy),
        (hx, hy),
        (-hx, hy)
    ]
    
    # Rotate and translate
    rotated_corners = []
    for cx, cy in corners:
        rx, ry = rotate_point(cx, cy, yaw)
        rotated_corners.append((x + rx, y + ry))
    
    return rotated_corners


def generate_svg_floor_plan(world_file, output_svg):
    """Generate SVG floor plan from world file."""
    # Parse the XML
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    world = root.find('.//world')
    if world is None:
        print("No world element found")
        return
    
    models = world.findall('model')
    
    # Collect all rectangles
    rectangles = []
    labels = []
    
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')
    
    for model in models:
        model_name = model.get('name', 'unnamed')
        
        if 'ground_plane' in model_name.lower() or 'ramp' in model_name.lower():
            continue
        
        model_pose_elem = model.find('pose')
        model_pose = parse_pose(model_pose_elem.text if model_pose_elem is not None else None)
        
        links = model.findall('.//link')
        
        for link in links:
            collision = link.find('.//collision/geometry/box')
            
            if collision is not None:
                size_elem = collision.find('size')
                if size_elem is not None:
                    size = parse_box_size(size_elem.text)
                    
                    # Get corners for 2D projection (top-down view)
                    corners = get_rectangle_corners(size[0], size[1], model_pose)
                    
                    # Update bounds
                    for cx, cy in corners:
                        min_x = min(min_x, cx)
                        max_x = max(max_x, cx)
                        min_y = min(min_y, cy)
                        max_y = max(max_y, cy)
                    
                    # Determine color based on type
                    if 'column' in model_name.lower():
                        color = '#6464FF'  # Blue
                        stroke = '#4040CC'
                    elif 'ramp' in model_name.lower():
                        color = '#808080'  # Gray
                        stroke = '#606060'
                    else:  # walls
                        color = '#333333'  # Dark gray
                        stroke = '#000000'
                    
                    rectangles.append({
                        'name': model_name,
                        'corners': corners,
                        'color': color,
                        'stroke': stroke,
                        'size': size,
                        'pose': model_pose
                    })
    
    # Calculate dimensions
    width = max_x - min_x
    height = max_y - min_y
    margin = 2.0  # 2 meter margin
    
    # SVG dimensions (scale: 50 pixels per meter)
    scale = 50
    svg_width = int((width + 2 * margin) * scale)
    svg_height = int((height + 2 * margin) * scale)
    
    # Start SVG
    svg_lines = []
    svg_lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    svg_lines.append(f'<svg width="{svg_width}" height="{svg_height}" xmlns="http://www.w3.org/2000/svg">')
    svg_lines.append('  <!-- Floor Plan -->')
    svg_lines.append(f'  <rect width="{svg_width}" height="{svg_height}" fill="white"/>')
    
    # Add grid (optional)
    svg_lines.append('  <!-- Grid (1m spacing) -->')
    svg_lines.append('  <g stroke="#E0E0E0" stroke-width="0.5">')
    for i in range(int(min_x - margin), int(max_x + margin) + 1):
        x = (i - min_x + margin) * scale
        svg_lines.append(f'    <line x1="{x}" y1="0" x2="{x}" y2="{svg_height}"/>')
    for j in range(int(min_y - margin), int(max_y + margin) + 1):
        y = (j - min_y + margin) * scale
        svg_lines.append(f'    <line x1="0" y1="{y}" x2="{svg_width}" y2="{y}"/>')
    svg_lines.append('  </g>')
    
    # Transform function (flip Y to match top-down view)
    def transform_x(x):
        return (x - min_x + margin) * scale
    
    def transform_y(y):
        # Flip Y-axis: subtract from total height to invert
        return svg_height - (y - min_y + margin) * scale
    
    # Draw rectangles
    svg_lines.append('  <!-- Building Elements -->')
    for rect in rectangles:
        corners = rect['corners']
        points = ' '.join([f"{transform_x(cx)},{transform_y(cy)}" for cx, cy in corners])
        
        svg_lines.append(f'    <polygon points="{points}" ')
        svg_lines.append(f'             fill="{rect["color"]}" ')
        svg_lines.append(f'             stroke="{rect["stroke"]}" ')
        svg_lines.append(f'             stroke-width="2"/>')
    
    # Add labels and dimensions
    svg_lines.append('  <!-- Labels -->')
    svg_lines.append('  <g font-family="Arial, sans-serif" font-size="24" font-weight="bold">')
    
    # Manual offset adjustments for labels
    label_offsets = {
        'column_1': (0, 40),   # Bottom of rectangle
        'column_2': (0, 40),   # Bottom of rectangle
        'column_3': (0, 40),   # Bottom of rectangle
        'column_4': (0, 40),   # Bottom of rectangle
        'column_5': (0, 40),   # Bottom of rectangle
        'column_6': (0, 40),   # Bottom of rectangle
        'column_7': (0, -40),  # Top of rectangle
        'wall_1': (0, -40),    # Move up
        'wall_5': (0, 40),     # Move down
    }
    
    for rect in rectangles:
        x, y = rect['pose'][0], rect['pose'][1]
        tx, ty = transform_x(x), transform_y(y)
        
        # Apply offset if specified
        offset_x, offset_y = label_offsets.get(rect['name'], (0, 0))
        tx += offset_x
        ty += offset_y
        
        # Add white background rectangle for text readability
        svg_lines.append(f'    <rect x="{tx - 60}" y="{ty - 25}" width="120" height="50" ')
        svg_lines.append(f'          fill="white" opacity="0.8" rx="5"/>')
        
        # Add name label (larger font)
        svg_lines.append(f'    <text x="{tx}" y="{ty - 5}" text-anchor="middle" ')
        svg_lines.append(f'          fill="black" font-size="18" font-weight="bold">{rect["name"]}</text>')
        
        # Add dimension label (larger font)
        size_x, size_y = rect['size'][0], rect['size'][1]
        dim_text = f'{size_x:.2f} × {size_y:.2f}m'
        svg_lines.append(f'    <text x="{tx}" y="{ty + 15}" text-anchor="middle" ')
        svg_lines.append(f'          fill="black" font-size="16">{dim_text}</text>')
    
    svg_lines.append('  </g>')
    
    # Add legend
    svg_lines.append('  <!-- Legend -->')
    svg_lines.append('  <g transform="translate(20, 20)">')
    svg_lines.append('    <rect width="200" height="80" fill="white" stroke="black" stroke-width="2" opacity="0.95"/>')
    svg_lines.append('    <rect x="15" y="15" width="30" height="20" fill="#333333" stroke="#000000" stroke-width="2"/>')
    svg_lines.append('    <text x="55" y="31" font-family="Arial" font-size="18">Walls</text>')
    svg_lines.append('    <rect x="15" y="40" width="30" height="20" fill="#6464FF" stroke="#4040CC" stroke-width="2"/>')
    svg_lines.append('    <text x="55" y="56" font-family="Arial" font-size="18">Columns</text>')
    svg_lines.append('  </g>')
    
    # Add scale indicator
    scale_length = 5  # 5 meters
    svg_lines.append('  <!-- Scale -->')
    svg_lines.append(f'  <g transform="translate({svg_width - 250}, {svg_height - 50})">')
    svg_lines.append('    <line x1="0" y1="15" x2="0" y2="0" stroke="black" stroke-width="3"/>')
    svg_lines.append(f'    <line x1="0" y1="7" x2="{scale_length * scale}" y2="7" stroke="black" stroke-width="3"/>')
    svg_lines.append(f'    <line x1="{scale_length * scale}" y1="15" x2="{scale_length * scale}" y2="0" stroke="black" stroke-width="3"/>')
    svg_lines.append(f'    <text x="{scale_length * scale / 2}" y="35" text-anchor="middle" font-family="Arial" font-size="18" font-weight="bold">{scale_length}m</text>')
    svg_lines.append('  </g>')
    
    # Add title
    svg_lines.append('  <!-- Title -->')
    svg_lines.append(f'  <text x="{svg_width/2}" y="40" text-anchor="middle" font-family="Arial" font-size="28" font-weight="bold">Floor Plan - Castelldefels Indoors</text>')
    
    svg_lines.append('</svg>')
    
    # Write to file
    with open(output_svg, 'w') as f:
        f.write('\n'.join(svg_lines))
    
    print(f"SVG floor plan saved to: {output_svg}")
    print(f"Dimensions: {width:.2f}m × {height:.2f}m")
    print(f"SVG size: {svg_width} × {svg_height} pixels")
    print(f"\nElements in floor plan:")
    for rect in rectangles:
        print(f"  {rect['name']}: {rect['size'][0]:.2f}m × {rect['size'][1]:.2f}m")


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    world_file = os.path.join(script_dir, "castelldefels_indoors_empty.world")
    output_svg = os.path.join(script_dir, "floor_plan.svg")
    
    print(f"Generating floor plan from: {world_file}")
    print(f"Output: {output_svg}\n")
    
    generate_svg_floor_plan(world_file, output_svg)
    
    print("\nTo convert to PNG, install inkscape and run:")
    print(f"  inkscape {output_svg} --export-type=png --export-filename=floor_plan.png")
