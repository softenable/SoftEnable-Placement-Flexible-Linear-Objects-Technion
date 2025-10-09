"""
Flexible Linear Object Placement Planner

This module provides the main entry point for planning the placement of flexible linear objects.
The planner uses a three-phase approach to compute displacement paths for soft objects.
"""

import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
import utils as hf
import node_class
import displacement as dis


class PlacementPlanner:
    """Main planner class for flexible linear object placement."""
    
    def __init__(self, urdf_path="urdf/softenable.urdf", ik_solver='fast'):
        """
        Initialize the placement planner.
        
        Args:
            urdf_path (str): Path to the robot URDF file
            ik_solver (str): IK solver type ('legacy', 'fast', or 'robust')
        """
        self.urdf_path = urdf_path
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.q_start = pin.neutral(self.model)
        self.ik_solver = ik_solver
        
        # Performance tracking
        self.planning_times = []
        self.ik_stats = {}
    
    def plan_placement(self, start_pose, target_poses, object_params, env_params, direction="right", profile=True):
        """
        Plan the placement of a flexible linear object.
        
        Args:
            start_pose (dict): Starting pose with keys 'position', 'orientation'
            target_poses (list): List of target poses for the three sub-problems
            object_params (dict): Object parameters including 'L' (length)
            env_params (dict): Environment parameters including friction coefficients
            direction (str): Movement direction ("right" or "left")
            profile (bool): Whether to collect performance statistics
            
        Returns:
            tuple: (success, paths, execution_time) where paths contains the three sub-problem paths
        """
        # Extract parameters
        L = object_params['L']
        mu_fric = env_params.get('mu_fric', 0.1)
        h_table = env_params.get('h_table', 0.0)
        alpha = env_params.get('alpha', 0.6)
        
        # Set direction-dependent parameters
        if direction == "right":
            C = 1/4
            phi_0_T2 = 0
        elif direction == "left":
            C = 3/4
            phi_0_T2 = np.pi
        else:
            raise ValueError("Direction must be 'right' or 'left'")
        
        # Create target and start nodes for sub-problem 1
        target_1_pose = target_poses[0]
        theta_S1 = np.pi/2
        theta_T1 = 0
        L_gal_S = 2*L
        
        target_1 = node_class.Target(
            target_1_pose['position'][0], target_1_pose['position'][1], target_1_pose['position'][2],
            np.pi/2, theta_T1, 1, C*L_gal_S, L_gal_S, 0, L, direction
        )
        
        start_1 = node_class.Node(
            target_1, start_pose['position'][0], start_pose['position'][1], start_pose['position'][2],
            np.pi/2, theta_S1, 1, C*L_gal_S, L_gal_S, 0, [], alpha, L, direction
        )
        
        # Create targets for sub-problems 2 and 3
        target_2 = node_class.Target(
            target_1.X0, target_1.Y0, target_1.Z0, phi_0_T2, target_1.theta,
            0, C*3.5*L, 3.5*L, 0, L, direction
        )
        
        l_T3 = 0.5*L
        target_3 = node_class.Target(
            target_2.X0, target_2.Y0, target_2.Z0, phi_0_T2, target_2.theta,
            0, C*4.5*(L - l_T3), 4.5*(L - l_T3), l_T3, L, direction
        )
        
        # Execute planning with timing
        t_start = time.time()
        
        try:
            # Reset IK solver stats if using new solver
            if self.ik_solver != 'legacy':
                ik_solver = hf.get_ik_solver(self.urdf_path)
                ik_solver.reset_stats()
            
            print(f"Using IK solver: {self.ik_solver}")
            
            # Sub-problem 1
            print("Solving sub-problem 1...")
            t1_start = time.time()
            path_1 = dis.displacement(start_1, target_1, L, mu_fric, direction, h_table, alpha, self.urdf_path, self.q_start, 1, self.ik_solver)
            t1_time = time.time() - t1_start
            print(f"Sub-problem 1 completed in {t1_time:.3f}s")
            
            if not path_1:
                return False, None, 0
            
            # Sub-problem 2  
            print("Solving sub-problem 2...")
            t2_start = time.time()
            path_2 = dis.displacement(path_1[-1], target_2, L, mu_fric, direction, h_table, alpha, self.urdf_path, path_1[-1].jointsPose, 2, self.ik_solver)
            t2_time = time.time() - t2_start
            print(f"Sub-problem 2 completed in {t2_time:.3f}s")
            
            if not path_2:
                return False, None, 0
            
            # Sub-problem 3
            print("Solving sub-problem 3...")
            t3_start = time.time()
            path_3 = dis.displacement(path_2[-1], target_3, L, mu_fric, direction, h_table, alpha, self.urdf_path, path_2[-1].jointsPose, 3, self.ik_solver)
            t3_time = time.time() - t3_start
            print(f"Sub-problem 3 completed in {t3_time:.3f}s")
            
            if not path_3:
                return False, None, 0
            
            t_end = time.time()
            execution_time = t_end - t_start
            
            # Collect performance statistics
            if profile:
                timing_stats = {
                    'total_time': execution_time,
                    'subproblem_1_time': t1_time,
                    'subproblem_2_time': t2_time,
                    'subproblem_3_time': t3_time,
                    'path_lengths': [len(path_1), len(path_2), len(path_3)]
                }
                
                if self.ik_solver != 'legacy':
                    ik_solver = hf.get_ik_solver(self.urdf_path)
                    ik_stats = ik_solver.get_performance_stats()
                    timing_stats['ik_stats'] = ik_stats
                
                self.planning_times.append(timing_stats)
                
                print(f"\nPerformance Summary:")
                print(f"  Total planning time: {execution_time:.3f}s")
                print(f"  Path lengths: {timing_stats['path_lengths']}")
                
                if 'ik_stats' in timing_stats and timing_stats['ik_stats']:
                    ik_stats = timing_stats['ik_stats']
                    print(f"  IK solver stats:")
                    print(f"    Total IK solves: {ik_stats['total_solves']}")
                    print(f"    Average IK time: {ik_stats['avg_solve_time']:.6f}s")
                    print(f"    Total IK time: {ik_stats['total_time']:.3f}s")
                    print(f"    Average iterations: {ik_stats['avg_iterations']:.1f}")
            
            return True, [path_1, path_2, path_3], execution_time
            
        except Exception as e:
            print(f"Planning failed: {e}")
            import traceback
            traceback.print_exc()
            return False, None, 0
    
    def visualize_paths(self, paths, object_params, direction="right"):
        """
        Visualize the computed paths.
        
        Args:
            paths (list): List of paths from plan_placement
            object_params (dict): Object parameters
            direction (str): Movement direction
        """
        if not paths:
            print("No paths to visualize")
            return
        
        L = object_params['L']
        path_1, path_2, path_3 = paths
        
        # Get smooth paths for visualization
        skip = 8
        num_t = 8
        t = np.linspace(0, 1, num_t)
        
        print("Generating smooth visualization paths...")
        x1, y1, z1, phi_l_1, theta_1, l1, q1 = hf.get_smooth_path_displacement(path_1, skip, t, L, direction, self.urdf_path, path_1[0].jointsPose, self.ik_solver)
        x2, y2, z2, phi_l_2, theta_2, l2, q2 = hf.get_smooth_path_displacement(path_2, skip, t, L, direction, self.urdf_path, path_2[0].jointsPose, self.ik_solver)
        x3, y3, z3, phi_l_3, theta_3, l3, q3 = hf.get_smooth_path_displacement(path_3, skip, t, L, direction, self.urdf_path, path_3[0].jointsPose, self.ik_solver)
        
        # Plot object shapes
        all_paths = path_1 + path_2 + path_3
        a = 1 if direction == "right" else -1
        
        plt.figure('Object Shapes')
        for n in all_paths:
            xs, zs, phi_s = hf.get_shape(n.X0 + a*n.l, n.Z0, n.phi_0, n.mu, n.L_gal, n.S0, np.linspace(0, L-n.l))
            plt.plot(xs, zs)
        plt.grid()
        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.title('Flexible Object Displacement Path')
        plt.show()
    
    def get_performance_stats(self):
        """Get planning performance statistics."""
        if not self.planning_times:
            return {}
        
        stats = {}
        for key in ['total_time', 'subproblem_1_time', 'subproblem_2_time', 'subproblem_3_time']:
            times = [run[key] for run in self.planning_times]
            stats[key] = {
                'mean': np.mean(times),
                'std': np.std(times),
                'min': np.min(times),
                'max': np.max(times)
            }
        
        # IK statistics if available
        if self.planning_times and 'ik_stats' in self.planning_times[-1]:
            stats['ik_stats'] = self.planning_times[-1]['ik_stats']
        
        return stats
    
    def benchmark_ik_solvers(self, test_poses=None, num_runs=3):
        """
        Benchmark different IK solvers on the current robot model.
        
        Args:
            test_poses: List of test poses, if None uses default poses
            num_runs: Number of runs for benchmarking
        """
        if test_poses is None:
            # Generate some test poses around the workspace
            test_poses = [
                [0.4, 0.2, 0.1, 0, np.pi/2],    # Standard pose
                [0.5, 0.3, 0.05, np.pi/4, 0],   # Different orientation
                [0.3, 0.1, 0.15, -np.pi/4, np.pi/3],  # Another pose
                [0.6, 0.0, 0.08, np.pi/2, -np.pi/4],  # Edge of workspace
            ]
        
        print("Benchmarking IK solvers...")
        results = hf.profile_ik_solvers(self.urdf_path, test_poses, self.q_start, num_runs)
        return results


def create_default_scenario():
    """Create a default test scenario."""
    # Object parameters
    object_params = {'L': 0.3}
    
    # Environment parameters
    env_params = {
        'mu_fric': 0.1,
        'h_table': 0.0,
        'alpha': 0.6
    }
    
    # Start pose
    start_pose = {
        'position': [0.4, 0.2, 0.09],  # h_table + 0.3*L
        'orientation': np.pi/2
    }
    
    # Target poses (only first one is used, others are computed automatically)
    target_poses = [{
        'position': [0.5, 0.2, 0.0],  # h_table
        'orientation': 0
    }]
    
    return start_pose, target_poses, object_params, env_params


def main():
    """Main entry point for the placement planner."""
    parser = argparse.ArgumentParser(description='Flexible Linear Object Placement Planner')
    parser.add_argument('--urdf', type=str, default='urdf/softenable.urdf', help='Path to robot URDF file')
    parser.add_argument('--direction', type=str, default='right', choices=['right', 'left'], help='Movement direction')
    parser.add_argument('--length', type=float, default=0.3, help='Object length')
    parser.add_argument('--visualize', action='store_true', default=True, help='Show visualization')
    parser.add_argument('--ik-solver', type=str, default='fast', choices=['legacy', 'fast', 'robust'], 
                       help='IK solver to use')
    parser.add_argument('--benchmark', action='store_true', help='Run IK solver benchmark')
    parser.add_argument('--profile', action='store_true', default=True, help='Enable performance profiling')
    
    args = parser.parse_args()
    
    # Create planner with specified IK solver
    planner = PlacementPlanner(args.urdf, args.ik_solver)
    
    # Run benchmark if requested
    if args.benchmark:
        print("Running IK solver benchmark...")
        benchmark_results = planner.benchmark_ik_solvers()
        return 0
    
    # Use default scenario (can be extended to load from config files)
    start_pose, target_poses, object_params, env_params = create_default_scenario()
    object_params['L'] = args.length
    
    print("Starting placement planning...")
    print(f"Object length: {object_params['L']}")
    print(f"Direction: {args.direction}")
    print(f"IK Solver: {args.ik_solver}")
    
    # Plan placement
    success, paths, execution_time = planner.plan_placement(
        start_pose, target_poses, object_params, env_params, args.direction, args.profile
    )
    
    if success:
        print(f"\nPlanning successful! Total execution time: {execution_time:.3f} seconds")
        
        # Show detailed performance stats if profiling is enabled
        if args.profile:
            stats = planner.get_performance_stats()
            if stats:
                print(f"\nDetailed Performance Statistics:")
                print(f"  Planning time breakdown:")
                for key in ['subproblem_1_time', 'subproblem_2_time', 'subproblem_3_time']:
                    if key in stats:
                        print(f"    {key}: {stats[key]['mean']:.3f}s")
        
        if args.visualize:
            planner.visualize_paths(paths, object_params, args.direction)
    else:
        print("Planning failed!")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())







