import os
import json
import xml.etree.ElementTree as ET
import argparse
import math

"""
HƯỚNG DẪN SỬ DỤNG:
Script này dùng để tạo file detector_config.json từ thông tin mạng lưới SUMO (network) và file định nghĩa detector.

Cách sử dụng:
1. Đảm bảo bạn đang ở thư mục gốc của dự án (PC_Algo_v2).
2. Chạy script bằng lệnh python:
   python tools/generate_detector_config.py

Tùy chọn dòng lệnh:
  --net-file <path>: Đường dẫn đến file .net.xml (Mặc định: src/network_test/zurich.net.xml)
  --add-file <path>: Đường dẫn đến file detector.add.xml (Mặc định: src/network_test/detector.add.xml)
  --output <path>: Đường dẫn file output json (Mặc định: detector_config_generated.json)
  --boundary-buffer <float>: Khoảng cách (mét) từ biên mạng lưới để coi một nút là nút biên (Mặc định: 50.0)

Logic lọc nút giao biên (Outer Intersections):
- Script sẽ xác định các "Nút biên" (Boundary Nodes) dựa trên:
  1. Loại nút là "dead_end".
  2. Tọa độ nút nằm gần biên của mạng lưới (convBoundary) trong khoảng buffer.
  3. Nút không có cạnh đi vào (Source node).
- Một nút giao (Traffic Light) được chọn vào `solver_input_detectors` nếu nó có ít nhất một detector (E1 hoặc E2) nằm trên cạnh đi vào từ một "Nút biên".
"""

def parse_detectors(add_file):
    """
    Parse detector.add.xml to map lanes to detectors.
    Returns:
        lane_to_e1: dict {lane_id: [e1_id, ...]}
        lane_to_e2: dict {lane_id: [e2_id, ...]}
        all_e1: list of all e1 ids
        all_e2: list of all e2 ids
    """
    tree = ET.parse(add_file)
    root = tree.getroot()
    
    lane_to_e1 = {}
    lane_to_e2 = {}
    all_e1 = []
    all_e2 = []

    # Parse E1 (inductionLoop)
    for detector in root.findall('inductionLoop'):
        det_id = detector.get('id')
        lane_id = detector.get('lane')
        all_e1.append(det_id)
        if lane_id not in lane_to_e1:
            lane_to_e1[lane_id] = []
        lane_to_e1[lane_id].append(det_id)

    # Parse E2 (laneAreaDetector)
    for detector in root.findall('laneAreaDetector'):
        det_id = detector.get('id')
        lane_id = detector.get('lane')
        all_e2.append(det_id)
        if lane_id not in lane_to_e2:
            lane_to_e2[lane_id] = []
        lane_to_e2[lane_id].append(det_id)
        
    return lane_to_e1, lane_to_e2, all_e1, all_e2

def parse_network(net_file):
    """
    Parse .net.xml to get intersections, phases, connections, nodes, and edges.
    """
    tree = ET.parse(net_file)
    root = tree.getroot()
    
    intersections = {}
    tl_connections = {}
    nodes = {}
    edges = {}
    
    # Parse Location for Boundary
    location = root.find('location')
    conv_boundary = [0, 0, 0, 0]
    if location is not None:
        boundary_str = location.get('convBoundary')
        if boundary_str:
            conv_boundary = [float(x) for x in boundary_str.split(',')] # minX, minY, maxX, maxY

    # Parse Junctions (Nodes)
    for junction in root.findall('junction'):
        jid = junction.get('id')
        if jid.startswith(':'): continue # Skip internal junctions
        
        nodes[jid] = {
            'x': float(junction.get('x')),
            'y': float(junction.get('y')),
            'type': junction.get('type'),
            'incLanes': junction.get('incLanes', '').split()
        }

    # Parse Edges
    for edge in root.findall('edge'):
        eid = edge.get('id')
        if eid.startswith(':'): continue # Skip internal edges
        
        edges[eid] = {
            'from': edge.get('from'),
            'to': edge.get('to')
        }

    # Parse Traffic Light Logics
    for tl in root.findall('tlLogic'):
        tl_id = tl.get('id')
        phases = []
        for phase in tl.findall('phase'):
            state = phase.get('state')
            duration = phase.get('duration')
            phases.append({'state': state, 'duration': duration})
        intersections[tl_id] = phases

    # Parse Connections
    for conn in root.findall('connection'):
        tl_id = conn.get('tl')
        if tl_id:
            link_index = int(conn.get('linkIndex'))
            from_edge = conn.get('from')
            from_lane_idx = conn.get('fromLane')
            lane_id = f"{from_edge}_{from_lane_idx}"
            
            if tl_id not in tl_connections:
                tl_connections[tl_id] = {}
            if link_index not in tl_connections[tl_id]:
                tl_connections[tl_id][link_index] = []
            tl_connections[tl_id][link_index].append(lane_id)
            
    return intersections, tl_connections, nodes, edges, conv_boundary

def is_boundary_node(node_id, node_data, conv_boundary, buffer=50.0):
    """
    Check if a node is a boundary node.
    """
    # Check type
    if node_data['type'] == 'dead_end':
        return True
    
    # Check coordinates
    x, y = node_data['x'], node_data['y']
    minX, minY, maxX, maxY = conv_boundary
    
    if (x <= minX + buffer) or (x >= maxX - buffer) or \
       (y <= minY + buffer) or (y >= maxY - buffer):
        return True
        
    # Check if source node (no incoming lanes - simplified check via incLanes attribute)
    # Note: incLanes might be empty for source nodes
    if not node_data['incLanes'] or node_data['incLanes'] == ['']:
        return True

    return False

def get_edge_from_lane(lane_id):
    # Lane ID is usually edgeID_index
    return "_".join(lane_id.split("_")[:-1])

def generate_config(net_file, add_file, output_file, boundary_buffer, target_intersections=None):
    print(f"Reading network from: {net_file}")
    print(f"Reading detectors from: {add_file}")

    lane_to_e1, lane_to_e2, all_e1, all_e2 = parse_detectors(add_file)
    intersections, tl_connections, nodes, edges, conv_boundary = parse_network(net_file)

    print(f"Network Boundary: {conv_boundary}")
    
    if target_intersections:
        print(f"Filtering for specific intersections: {target_intersections}")
    else:
        print(f"Filtering for outer intersections (buffer={boundary_buffer}m)")
    
    # Identify Boundary Nodes
    boundary_nodes = set()
    for nid, data in nodes.items():
        if is_boundary_node(nid, data, conv_boundary, boundary_buffer):
            boundary_nodes.add(nid)
    
    print(f"Found {len(boundary_nodes)} boundary nodes.")

    solver_input_detectors = {}

    for tl_id, phases in intersections.items():
        if tl_id not in tl_connections:
            continue

        tl_data = {"phases": {}}
        green_phases = []
        
        is_outer_intersection = False

        for i, phase in enumerate(phases):
            state = phase.get('state')
            has_relevant_green = False
            
            phase_detectors = {
                "queue_detectors": [], # E2
                "queue2_detector": []  # E1
            }
            
            for link_idx, char in enumerate(state):
                if char.lower() == 'g':
                    if link_idx in tl_connections[tl_id]:
                        lanes = tl_connections[tl_id][link_idx]
                        for lane in lanes:
                            # Check if this lane comes from a boundary node
                            edge_id = get_edge_from_lane(lane)
                            if edge_id in edges:
                                from_node = edges[edge_id]['from']
                                if from_node in boundary_nodes:
                                    # Check if there are detectors on this lane
                                    has_detectors_on_lane = (lane in lane_to_e2) or (lane in lane_to_e1)
                                    if has_detectors_on_lane:
                                        is_outer_intersection = True

                            # Add E2 detectors
                            if lane in lane_to_e2:
                                for det in lane_to_e2[lane]:
                                    if det not in phase_detectors["queue_detectors"]:
                                        phase_detectors["queue_detectors"].append(det)
                                        has_relevant_green = True
                            
                            # Add E1 detectors
                            if lane in lane_to_e1:
                                for det in lane_to_e1[lane]:
                                    if det not in phase_detectors["queue2_detector"]:
                                        phase_detectors["queue2_detector"].append(det)
                                        has_relevant_green = True

            if has_relevant_green:
                green_phases.append(phase_detectors)

        # Determine if we should include this intersection
        should_include = False
        if target_intersections is not None and len(target_intersections) > 0:
            if tl_id in target_intersections:
                should_include = True
        else:
            if is_outer_intersection:
                should_include = True

        # Only add this intersection if it meets criteria
        if should_include and green_phases:
            tl_data["phases"]["p"] = green_phases[0]
            if len(green_phases) > 1:
                tl_data["phases"]["s"] = []
                for i in range(1, len(green_phases)):
                    phase_obj = green_phases[i]
                    phase_obj["comment"] = f"Secondary Phase {i}"
                    tl_data["phases"]["s"].append(phase_obj)
            else:
                 tl_data["phases"]["s"] = []

            solver_input_detectors[tl_id] = tl_data

    output_data = {
        "metadata": {
            "description": "Configuration for detectors generated from network.",
            "version": "1.2",
            "comment": "Generated by tools/generate_detector_config.py"
        },
        "algorithm_input_detectors": {
            "description": "List of all E2 detectors.",
            "detector_ids": all_e2
        },
        "mfd_input_flow_detectors": {
            "description": "List of all E1 detectors.",
            "detector_ids": all_e1
        },
        "solver_input_detectors": {
            "description": "Mapping of detectors to intersections and phases.",
            "intersections": solver_input_detectors
        }
    }

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"Successfully generated config at: {output_file}")
    print(f"Total intersections included: {len(solver_input_detectors)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate detector_config.json from SUMO network.")
    parser.add_argument("--net-file", default="src/network_test/zurich.net.xml", help="Path to .net.xml file")
    parser.add_argument("--add-file", default="src/network_test/detector.add.xml", help="Path to detector.add.xml file")
    parser.add_argument("--output", default="detector_config_generated.json", help="Path to output json file")
    parser.add_argument("--boundary-buffer", type=float, default=50.0, help="Buffer distance from network boundary to consider a node as boundary node")
    parser.add_argument("--target-intersections", help="Comma separated list of intersection IDs to include. If provided, ignores boundary logic.")
    
    args = parser.parse_args()
    
    targets = None
    if args.target_intersections:
        targets = args.target_intersections.split(',')
    
    generate_config(args.net_file, args.add_file, args.output, args.boundary_buffer, targets)
