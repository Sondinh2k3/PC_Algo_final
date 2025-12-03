import os
import json
import xml.etree.ElementTree as ET
import argparse

"""
HƯỚNG DẪN SỬ DỤNG:
Script này dùng để tạo file intersection_config.json từ thông tin mạng lưới SUMO (network).

Cách sử dụng:
1. Đảm bảo bạn đang ở thư mục gốc của dự án (PC_Algo_v2).
2. Chạy script bằng lệnh python:
   python tools/generate_intersection_config.py --target-intersections <list_of_ids>

Tùy chọn dòng lệnh:
  --net-file <path>: Đường dẫn đến file .net.xml (Mặc định: src/network_test/zurich.net.xml)
  --output <path>: Đường dẫn file output json (Mặc định: intersection_config_generated.json)
  --target-intersections <list>: Danh sách ID các nút giao cần điều khiển, phân cách bằng dấu phẩy (Bắt buộc).
                                 Ví dụ: --target-intersections zurich_ts_1,zurich_ts_15

Logic:
- Script sẽ đọc file network để lấy thông tin về các Traffic Lights (TL) được chỉ định.
- Cấu trúc file output sẽ bao gồm:
  - metadata: Thông tin chung.
  - traffic_lights: Cấu hình pha đèn (phases, duration, state) lấy từ network.
  - intersections: Thông tin vị trí (x, y) và ID của nút giao.
  - optimization_parameters: Các tham số mặc định cho thuật toán tối ưu hóa (cycle_length, saturation_flow, etc.).
"""

def parse_network(net_file, target_ids):
    """
    Parse .net.xml to get traffic light logic and junction positions.
    """
    tree = ET.parse(net_file)
    root = tree.getroot()
    
    traffic_lights = {}
    intersections = {}
    
    # Parse Traffic Light Logics
    for tl in root.findall('tlLogic'):
        tl_id = tl.get('id')
        if tl_id not in target_ids:
            continue
            
        phases = []
        total_cycle = 0
        for phase in tl.findall('phase'):
            state = phase.get('state')
            duration = float(phase.get('duration'))
            phases.append({'duration': duration, 'state': state})
            total_cycle += duration
            
        traffic_lights[tl_id] = {
            "type": tl.get('type', 'static'),
            "phases": phases,
            "total_cycle": int(total_cycle)
        }

    # Parse Junctions (Nodes) to get coordinates
    # Note: In SUMO, a TL system might control multiple junctions or a single junction.
    # Here we assume the TL ID corresponds to a Junction ID or we find the junction controlled by it.
    # For simplicity, we look for junctions with id == tl_id first.
    
    # Create a mapping of TL ID to Junction ID(s) if needed, but often they match or we use the TL ID as key.
    for junction in root.findall('junction'):
        jid = junction.get('id')
        if jid in target_ids:
            intersections[jid] = {
                "id": jid,
                "traffic_light_id": jid,
                "type": "traffic_light",
                "x": float(junction.get('x')),
                "y": float(junction.get('y'))
            }
            
    return traffic_lights, intersections

def generate_config(net_file, output_file, target_intersections):
    print(f"Reading network from: {net_file}")
    print(f"Target intersections: {target_intersections}")

    target_ids = target_intersections.split(',')
    traffic_lights, intersections = parse_network(net_file, target_ids)
    
    # Check if we found all targets
    found_ids = set(traffic_lights.keys())
    missing_ids = set(target_ids) - found_ids
    if missing_ids:
        print(f"Warning: Could not find the following Traffic Lights in network: {missing_ids}")

    # Generate Optimization Parameters Data
    intersection_data = {}
    for tl_id in traffic_lights.keys():
        # Default logic to assign phases to 'p' (primary) and 's' (secondary)
        # This is a heuristic: Assume Green phases are relevant.
        # We need to map indices. For now, we'll create a placeholder structure 
        # that the user might need to fine-tune manually, or we use simple logic.
        
        # Simple heuristic: First long green phase is P, others are S.
        phases_list = traffic_lights[tl_id]['phases']
        p_phase = None
        s_phases = []
        
        for i, phase in enumerate(phases_list):
            # Heuristic: Green phase usually has 'G' or 'g' and duration > 5
            if ('G' in phase['state'] or 'g' in phase['state']) and phase['duration'] > 5:
                if p_phase is None:
                    p_phase = {
                        "phase_indices": [i],
                        "saturation_flow": 0.5,
                        "turn_in_ratio": 0.5,
                        "queue_length": 20
                    }
                else:
                    s_phases.append({
                        "phase_indices": [i],
                        "saturation_flow": 0.5,
                        "turn_in_ratio": 0.5,
                        "queue_length": 10
                    })
        
        # If no green phase found (unlikely), just take 0 as P
        if p_phase is None and phases_list:
             p_phase = {
                "phase_indices": [0],
                "saturation_flow": 0.5,
                "turn_in_ratio": 0.5,
                "queue_length": 20
            }

        intersection_data[tl_id] = {
            "cycle_length": traffic_lights[tl_id]['total_cycle'],
            "phases": {
                "p": p_phase,
                "s": s_phases
            }
        }

    output_data = {
        "metadata": {
            "network_file": os.path.basename(net_file),
            "generated_at": "generated by tool",
            "total_intersections": len(traffic_lights),
            "total_traffic_lights": len(traffic_lights)
        },
        "traffic_lights": traffic_lights,
        "intersections": intersections,
        "optimization_parameters": {
            "intersection_ids": list(traffic_lights.keys()),
            "theta_1": 1.0,
            "theta_2": 1.0,
            "default_cycle_length": 90,
            "min_green_time": 15,
            "max_green_time": 75,
            "max_change": 10,
            "intersection_data": intersection_data
        }
    }

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"Successfully generated config at: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate intersection_config.json from SUMO network.")
    parser.add_argument("--net-file", default="src/network_test/zurich.net.xml", help="Path to .net.xml file")
    parser.add_argument("--output", default="intersection_config_generated.json", help="Path to output json file")
    parser.add_argument("--target-intersections", required=True, help="Comma separated list of intersection IDs to include.")
    
    args = parser.parse_args()
    
    generate_config(args.net_file, args.output, args.target_intersections)
