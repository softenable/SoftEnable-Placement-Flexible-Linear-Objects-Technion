#!/usr/bin/env python3
# © Copyright 2025, Ocado Group PLC. All the rights reserved.
"""
Placement Planner Runner

This script runs the main placement planner for flexible linear objects.
It provides a simple interface to execute the planner with different configurations.
"""

import sys
import os
import argparse
import numpy as np

# Add src directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
from main import PlacementPlanner

def main():
    """Main planner execution."""
    parser = argparse.ArgumentParser(description='Flexible Linear Object Placement Planner')
    parser.add_argument('--urdf', type=str, default='urdf/softenable.urdf',
                       help='Path to robot URDF file (default: urdf/softenable.urdf)')
    parser.add_argument('--solver', type=str, choices=['legacy', 'fast', 'robust'], 
                       default='fast', help='IK solver to use (default: fast)')
    parser.add_argument('--visualize', action='store_true',
                       help='Enable visualization of the planning process')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    print("🤖 Flexible Linear Object Placement Planner")
    print("=" * 50)
    print(f"URDF: {args.urdf}")
    print(f"IK Solver: {args.solver}")
    print(f"Visualization: {'Enabled' if args.visualize else 'Disabled'}")
    print("=" * 50)
    
    try:
        # Initialize planner
        planner = PlacementPlanner(urdf_path=args.urdf, ik_solver=args.solver)
        
        # Run planner with example parameters
        print("\n🚀 Starting placement planning...")
        
        # Example parameters - customize these based on your specific use case
        start_pose = {
            'position': [0.4, 0.2, 0.3 * 0.1],
            'orientation': np.pi/2  # theta angle in radians
        }

        target_poses = [
            {'position': [0.4, 0.2, 0.0], 'orientation': 0.0},
            # {'position': [0.7, 0.2, 0.5], 'orientation': 0.2},
            # {'position': [0.8, 0.3, 0.5], 'orientation': 0.3}
        ]
        
        object_params = {
            'L': 0.1  # Object length in meters
        }
        
        env_params = {
            'mu_fric': 0.1,     # Friction coefficient
            'h_table': 0.0,     # Table height
            'alpha': 0.6        # Object parameter
        }
        
        success, paths, execution_time = planner.plan_placement(
            start_pose=start_pose,
            target_poses=target_poses,
            object_params=object_params,
            env_params=env_params,
            direction="right",
            profile=True
        )
        
        if success:
            print(f"✅ Planning completed successfully in {execution_time:.3f}s!")
            print(f"Generated {len(paths)} path segments")
            if args.verbose:
                for i, path in enumerate(paths):
                    print(f"  Path {i+1}: {len(path)} waypoints")
        else:
            print("❌ Planning failed!")
            
    except Exception as e:
        print(f"❌ Error during planning: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()