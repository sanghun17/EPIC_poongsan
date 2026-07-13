#!/usr/bin/env python3
"""
PCD Boundary and Initial Position Analyzer
Extracts min/max coordinates and first point from binary PCD files
"""

import struct
import sys

def analyze_pcd(pcd_path):
    """
    Parse binary PCD file and extract:
    - Min/max XYZ coordinates
    - First point (SLAM start position)
    """
    print(f"\n=== Analyzing: {pcd_path} ===")

    with open(pcd_path, 'rb') as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline().decode('ascii').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        # Parse header
        num_points = 0
        for line in header_lines:
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
                break

        print(f"Total points: {num_points:,}")

        # Read binary data (each point is 3 floats: x, y, z)
        point_size = 12  # 3 floats * 4 bytes

        # Initialize min/max
        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')
        first_point = None

        # Read and process points
        print("Reading points... (this may take a while)")
        for i in range(num_points):
            data = f.read(point_size)
            if len(data) < point_size:
                break

            x, y, z = struct.unpack('fff', data)

            # Store first point
            if i == 0:
                first_point = (x, y, z)

            # Update min/max
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)

            # Progress indicator
            if (i + 1) % 1000000 == 0:
                print(f"  Processed {i+1:,} / {num_points:,} points...")

        print(f"✓ Analysis complete!")

        return {
            'num_points': num_points,
            'min': (min_x, min_y, min_z),
            'max': (max_x, max_y, max_z),
            'first': first_point
        }

def main():
    # Paths to PCD files
    resource_dir = "/home/ml/epic_poongsan/src/EPIC_poongsan/src/EPIC/src/MARSIM/map_generator/resource"

    maps = {
        'daejun_rail': f"{resource_dir}/daejun_rail.pcd",
        'bukang': f"{resource_dir}/bukang.pcd"
    }

    results = {}

    for map_name, pcd_path in maps.items():
        try:
            results[map_name] = analyze_pcd(pcd_path)
        except Exception as e:
            print(f"ERROR analyzing {map_name}: {e}")
            sys.exit(1)

    # Print summary
    print("\n" + "="*70)
    print("SUMMARY RESULTS")
    print("="*70)

    for map_name, data in results.items():
        print(f"\n📍 {map_name.upper()}")
        print(f"   Points: {data['num_points']:,}")
        print(f"   Boundary:")
        print(f"     box_0/down: [{data['min'][0]:.2f}, {data['min'][1]:.2f}, {data['min'][2]:.2f}]")
        print(f"     box_0/up:   [{data['max'][0]:.2f}, {data['max'][1]:.2f}, {data['max'][2]:.2f}]")
        print(f"   Initial Position (first point):")
        print(f"     init_x_: {data['first'][0]:.2f}")
        print(f"     init_y_: {data['first'][1]:.2f}")
        print(f"     init_z_: {data['first'][2]:.2f}")
        print(f"   Suggested init_z (first_z + 1.0m): {data['first'][2] + 1.0:.2f}")

    print("\n" + "="*70)

    # Save results to file for automated processing
    output_file = "/home/ml/epic_poongsan/src/EPIC_poongsan/pcd_analysis_results.txt"
    with open(output_file, 'w') as f:
        for map_name, data in results.items():
            f.write(f"{map_name}\n")
            f.write(f"box_down:{data['min'][0]:.2f},{data['min'][1]:.2f},{data['min'][2]:.2f}\n")
            f.write(f"box_up:{data['max'][0]:.2f},{data['max'][1]:.2f},{data['max'][2]:.2f}\n")
            f.write(f"init_pos:{data['first'][0]:.2f},{data['first'][1]:.2f},{data['first'][2] + 1.0:.2f}\n")

    print(f"\n✓ Results saved to: {output_file}")

if __name__ == "__main__":
    main()
