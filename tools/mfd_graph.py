#!/usr/bin/env python3
"""
MFD Graph Tool
==============
A simple tool to collect data from e1 (flow) and e2 (vehicle count) detectors
and create a Macroscopic Fundamental Diagram (MFD) graph.

Usage:
    python mfd_graph.py --sim-config <path> --detector-config <path>
"""

import os
import sys
import json
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml
import traci
import traci.exceptions
import argparse

# Add project root to system path
current_dir = os.path.dirname(os.path.abspath(__name__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)

from src.sumosim import SumoSim

# Check SUMO_HOME
if 'SUMO_HOME' not in os.environ:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

def load_config(sim_config_path, detector_config_path):
    """Load configuration files."""
    print("Loading configuration files...")
    
    # Load simulation config
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)['config']
    
    # Load detector config
    with open(detector_config_path, 'r') as f:
        detector_config = json.load(f)
    
    return sim_config, detector_config

def collect_data(sim_config, detector_config):
    """
    Collect and aggregate data for MFD (Flow vs. Accumulation).
    The function aggregates data over a 50-second interval.
    """
    print("Starting SUMO simulation...")
    
    # Get detector IDs
    e2_detectors = detector_config['algorithm_input_detectors']['detector_ids']  # For accumulation
    e1_detectors = detector_config['mfd_input_flow_detectors']['detector_ids']   # For flow
    
    print(f"Found {len(e2_detectors)} e2 detectors for accumulation")
    print(f"Found {len(e1_detectors)} e1 detectors for flow")
    
    # Initialize SUMO simulation
    sumo_sim = SumoSim(sim_config)
    
    data_points = []
    
    try:
        sumo_sim.start() 
        
        # Simulation parameters
        simulation_time = 6000
        step_length = float(sim_config.get('step_length', 1.0))
        total_steps = int(simulation_time / step_length)

        # 1. Đặt khoảng thời gian lấy mẫu là 50 giây
        sampling_interval = 50.0
        steps_per_sample = int(sampling_interval / step_length)
        
        accumulated_flow = 0
        accumulation_samples = []

        print(f"Running simulation for {simulation_time} seconds with {sampling_interval}s aggregation interval...")
        
        for step in range(total_steps):
            sumo_sim.step()
            
            # Luôn thu thập dữ liệu ở mỗi bước để tổng hợp
            # Lấy số xe hiện tại trong khu vực
            current_accumulation = 0
            for det_id in e2_detectors:
                try:
                    space_occupy = traci.lanearea.getLastIntervalOccupancy(det_id)
                    road_length = 80
                    num_lane = 1
                    average_length_of_vehicles = 2.5
                    accumulation = road_length * (num_lane / (100 * average_length_of_vehicles)) * space_occupy
                    current_accumulation += accumulation
                except traci.exceptions.TraCIException:
                    pass
            accumulation_samples.append(current_accumulation)
            
            # Cộng dồn lưu lượng xe đi qua
            for det_id in e1_detectors:
                try:
                    # Dù detector có chu kỳ 10s, việc cộng dồn mỗi step vẫn đảm bảo không mất dữ liệu
                    if (step + 1) % 10 == 0 and step > 0:
                        accumulated_flow += traci.inductionloop.getLastIntervalVehicleNumber(det_id)
                except traci.exceptions.TraCIException:
                    pass
            
            # Sau mỗi 50 giây, tổng hợp và ghi lại một điểm dữ liệu
            if (step + 1) % steps_per_sample == 0 and step > 0:
                
                # 2. Quay lại tính SỐ LƯỢNG XE TRUNG BÌNH trong 50s
                # Đây là cách biểu diễn 'tổng số lượng xe' một cách chính xác nhất cho một khoảng thời gian
                avg_accumulation = sum(accumulation_samples) / len(accumulation_samples) if accumulation_samples else 0
                accumulation = sum(accumulation_samples)
                # Tính lưu lượng (tổng số xe đi qua trong 50s, quy đổi ra giờ)
                flow_per_hour = accumulated_flow * (3600 / sampling_interval)
                
                # Lưu điểm dữ liệu đã được tổng hợp
                current_time = (step + 1) * step_length
                data_points.append({
                    'avg_accumulation': avg_accumulation,
                    'flow_per_hour': flow_per_hour,
                    'time': current_time
                })
                
                print(f"  Time {current_time}s: "
                      f"Avg_accumulation={avg_accumulation:.1f} vehicles, Flow={flow_per_hour:.1f} veh/h")
                
                # Reset các biến để bắt đầu chu kỳ 50s tiếp theo
                accumulated_flow = 0
                accumulation_samples = []
                
        print("Simulation completed successfully!")
        
    except Exception as e:
        print(f"Error during simulation: {e}")
        return None
    finally:
        if traci.isLoaded():
            sumo_sim.close()
    
    return data_points

def create_mfd_graph(data_points, output_dir):
    """Create MFD graph from collected data (Flow vs. Accumulation)."""
    if not data_points:
        print("No data collected. Cannot create graph.")
        return
    
    print("Creating MFD graph...")
    
    # Convert to DataFrame
    df = pd.DataFrame(data_points)
    
    # THAY ĐỔI: Sử dụng cột 'accumulation' thay vì 'vehicle_count'
    # Remove zero values and outliers
    df = df[(df['avg_accumulation'] > 0) & (df['flow_per_hour'] > 0)]
    
    if len(df) == 0:
        print("No valid data points after filtering.")
        return
    
    # Create the graph
    plt.figure(figsize=(12, 8))
    
    # Scatter plot
    plt.scatter(df['avg_accumulation'], df['flow_per_hour'], 
                alpha=0.6, s=30, color='blue', label='Data Points')
    
    # Add trend line
    if len(df) > 1:
        z = np.polyfit(df['avg_accumulation'], df['flow_per_hour'], 2)
        p = np.poly1d(z)
        x_trend = np.linspace(df['avg_accumulation'].min(), df['avg_accumulation'].max(), 100)
        plt.plot(x_trend, p(x_trend), 'r-', linewidth=2, label='Trend Line')
    
    # Customize graph with updated labels
    plt.title('Macroscopic Fundamental Diagram (MFD)', fontsize=16, fontweight='bold')
    plt.xlabel('Accumulation (Total Vehicles in Region)', fontsize=12)
    plt.ylabel('Flow Rate (vehicles/hour)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Add statistics with updated labels
    stats_text = f"""
    Data Points: {len(df)}
    Accumulation Range: {df['avg_accumulation'].min():.0f} - {df['avg_accumulation'].max():.0f} vehicles
    Flow Range: {df['flow_per_hour'].min():.1f} - {df['flow_per_hour'].max():.1f} veh/h
    Max Flow: {df['flow_per_hour'].max():.1f} veh/h
    """
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Save and show
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "mfd_graph.png")
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Graph saved to: {output_path}")
    
    # Show the graph
    plt.show()

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Create a Macroscopic Fundamental Diagram (MFD) graph.')
    parser.add_argument('--sim-config', type=str, default=os.path.join(project_root, 'src', 'config', 'simulation.yml'), help='Path to simulation.yml')
    parser.add_argument('--detector-config', type=str, default=os.path.join(project_root, 'src', 'config', 'detector_config.json'), help='Path to detector_config.json')
    parser.add_argument('--output-dir', type=str, default=os.path.join(project_root, 'output'), help='Directory to save the MFD graph')
    args = parser.parse_args()

    print("=" * 50)
    print("MFD Graph Tool")
    print("=" * 50)
    
    try:
        # Load configuration
        sim_config, detector_config = load_config(args.sim_config, args.detector_config)
        
        # Collect data
        data_points = collect_data(sim_config, detector_config)
        
        # Create graph
        if data_points:
            create_mfd_graph(data_points, args.output_dir)
            print("\nMFD analysis completed successfully!")
        else:
            print("Failed to collect data.")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
