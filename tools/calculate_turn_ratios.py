import xml.etree.ElementTree as ET
from collections import defaultdict
import os
import argparse

# ==============================================================================
# --- CONFIGURATION ---
# Vui lòng chỉnh sửa các giá trị trong phần này cho phù hợp với kịch bản của bạn
# ==============================================================================

# 2. ĐỊNH NGHĨA VÀNH ĐAI (PERIMETER)
# Đây là phần quan trọng nhất bạn cần định nghĩa.
# Dựa trên file grid.net.xml, mạng lưới có vẻ là một lưới 3x4 (A,B,C,D x 1,2,3).
# Giả sử khu vực trung tâm (khu vực được bảo vệ) là các nút B2, C2 và các đường nối giữa chúng.

# Liệt kê ID của các nút giao được coi là cổng vào (vành đai)
BOUNDARY_JUNCTIONS = {
    "B1", "C1", "D1",
    "B2", "C2", "D2",
    "B3", "C3", "D3"
}

# Liệt kê ID của các cạnh (đường) được coi là nằm "BÊN TRONG" khu vực trung tâm.
# Bất kỳ xe nào rẽ vào một trong các cạnh này từ một nút giao vành đai
# sẽ được tính là "đi vào chu vi" (turn in).
PERIMETER_EDGES = {
    # Các đường nối B2 và C2
    "B2C2", "C2B2",
    # Các đường nối B2 và B1, B3
    "B1B2", "B2B1", "B2B3", "B3B2",
    # Các đường nối C2 và C1, C3
    "C1C2", "C2C1", "C2C3", "C3C2",
    # Các đường nối C2 và D2
    "C2D2", "D2C2",
    # Các đường nối D2, D1 và D3
    "D1D2", "D2D1", "D2D3", "D3D2",
    # Các đường nối B1, C1 và D1
    "B1C1", "C1B1", "C1D1", "D1C1",
    # Các đường nối B3, C3 và D3
    "B3C3", "C3B3", "C3D3", "D3C3"
}


# ==============================================================================
# --- SCRIPT LOGIC ---
# Bạn không cần chỉnh sửa phần dưới này
# ==============================================================================

def analyze_turn_ratios(vehroute_file):
    """
    Phân tích file vehicle routes để tính toán tỷ lệ rẽ tại các nút giao vành đai.
    """
    # Cấu trúc dữ liệu để lưu trữ số đếm
    # turn_counts[from_edge][to_edge] = count
    turn_counts = defaultdict(lambda: defaultdict(int))

    # Kiểm tra file tồn tại
    if not os.path.exists(vehroute_file):
        print(f"LỖI: Không tìm thấy file '{vehroute_file}'.")
        print("Vui lòng kiểm tra lại đường dẫn và đảm bảo bạn đã chạy mô phỏng để tạo file output.")
        return

    print(f"Đang phân tích file: {vehroute_file}...")

    # Phân tích file XML một cách hiệu quả về bộ nhớ
    try:
        context = ET.iterparse(vehroute_file, events=('start', 'end'))
        context = iter(context)
        event, root = next(context)

        for event, elem in context:
            if event == 'end' and elem.tag == 'vehicle':
                route_elem = elem.find('route')
                if route_elem is not None and 'edges' in route_elem.attrib:
                    edges = route_elem.attrib['edges'].split()
                    # Lặp qua các cặp cạnh để xác định lượt rẽ
                    for i in range(len(edges) - 1):
                        from_edge = edges[i]
                        to_edge = edges[i+1]
                        turn_counts[from_edge][to_edge] += 1
                # Xóa phần tử đã xử lý để giải phóng bộ nhớ
                root.clear()
    except ET.ParseError as e:
        print(f"LỖI: File XML '{vehroute_file}' bị lỗi hoặc không đúng định dạng: {e}")
        return


    print("\n" + "="*50)
    print("KẾT QUẢ PHÂN TÍCH TỶ LỆ RẼ")
    print("="*50)

    # Xử lý và in kết quả
    # Lấy danh sách các cạnh đi vào nút giao vành đai
    all_from_edges = sorted(turn_counts.keys())
    
    processed_junctions = set()

    for from_edge in all_from_edges:
        # Giả định tên nút giao từ ID của cạnh (ví dụ: "B1C1" -> from "B1", to "C1")
        # Điều này có thể không chính xác 100% nếu quy ước đặt tên phức tạp
        from_node = from_edge.split("B")[-1].split("C")[-1].split("D")[-1].split("E")[-1][0]
        from_node_guess = from_edge[0] + from_node
        
        if from_node_guess not in BOUNDARY_JUNCTIONS:
            continue

        # Tính toán tổng số xe và số xe rẽ vào
        total_vehicles_from_edge = sum(turn_counts[from_edge].values())
        vehicles_turning_in = 0
        
        print(f"\n--- Nút giao: {from_node_guess} (từ cạnh {from_edge}) ---")
        print(f"  Tổng số xe: {total_vehicles_from_edge}")

        for to_edge, count in sorted(turn_counts[from_edge].items()):
            is_turn_in = "✅" if to_edge in PERIMETER_EDGES else "❌"
            if is_turn_in == "✅":
                vehicles_turning_in += count
            print(f"    - Rẽ vào cạnh '{to_edge}': {count} xe {is_turn_in}")

        if total_vehicles_from_edge > 0:
            turn_in_ratio = vehicles_turning_in / total_vehicles_from_edge
            print(f"  => TỔNG XE RẼ VÀO: {vehicles_turning_in}")
            print(f"  => TỶ LỆ RẼ VÀO (turn_in_ratio): {turn_in_ratio:.2f}")
        else:
            print("  => Không có xe nào đi qua cạnh này.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Phân tích file vehicle routes để tính toán tỷ lệ rẽ tại các nút giao vành đai.')
    parser.add_argument('vehroute_file', type=str, help='Đường dẫn đến file vehroutes.xml')
    args = parser.parse_args()
    analyze_turn_ratios(args.vehroute_file)