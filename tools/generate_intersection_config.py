#!/usr/bin/env python3
"""
Script tự động phân tích SUMO network và tạo file cấu hình intersection
Sử dụng: python tools/generate_intersection_config.py [network_file] [output_file]
"""

import sys
import os
import argparse
from pathlib import Path

# Thêm đường dẫn gốc của dự án vào sys.path để có thể import src
PROJECT_ROOT_PATH = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT_PATH))

# Import từ src
from src.data.intersection_analyzer import IntersectionAnalyzer

def main():
    # Xác định đường dẫn mặc định một cách bền vững
    DEFAULT_NET_FILE = PROJECT_ROOT_PATH / 'src' / 'PhuQuoc' / 'phuquoc.net.xml'
    DEFAULT_OUTPUT_FILE = PROJECT_ROOT_PATH / 'src' / 'config' / 'intersection_config.json'

    parser = argparse.ArgumentParser(description='Tạo cấu hình intersection từ SUMO network')
    parser.add_argument('network_file', nargs='?', default=str(DEFAULT_NET_FILE),
                       help=f'Đường dẫn đến file .net.xml (mặc định: {DEFAULT_NET_FILE})')
    parser.add_argument('output_file', nargs='?', default=str(DEFAULT_OUTPUT_FILE),
                       help=f'File output JSON (mặc định: {DEFAULT_OUTPUT_FILE})')
    parser.add_argument('--analyze-only', action='store_true',
                       help='Chỉ phân tích network, không tạo cấu hình mặc định')
    
    args = parser.parse_args()
    
    print("🚦 TẠO CẤU HÌNH INTERSECTION TỪ SUMO NETWORK")
    print("="*60)
    
    # Kiểm tra file network
    if not os.path.exists(args.network_file):
        print(f"❌ Không tìm thấy file network: {args.network_file}")
        print(f"💡 Sử dụng: python tools/generate_intersection_config.py [network_file] [output_file]")
        return False
    
    try:
        # Tạo analyzer
        analyzer = IntersectionAnalyzer(args.network_file)
        
        # Phân tích network
        print(f"📁 Đang phân tích network: {args.network_file}")
        network_data = analyzer.analyze_network()
        
        if not network_data:
            print("❌ Không tìm thấy intersection nào trong network")
            return False
        
        # Tạo cấu hình
        if args.analyze_only:
            print("🔍 Chế độ chỉ phân tích - không tạo cấu hình")
            print(f"Tìm thấy {len(network_data)} intersections:")
            for intersection_id, data in network_data.items():
                print(f"  - {intersection_id}: {data.get('type', 'unknown')}")
        else:
            print(f"📝 Đang tạo cấu hình: {args.output_file}")
            config_data = analyzer.generate_intersection_config(args.output_file)
            
            if config_data:
                print("✅ Tạo cấu hình thành công!")
                return True
            else:
                print("❌ Lỗi khi tạo cấu hình")
                return False
    
    except Exception as e:
        print(f"❌ Lỗi: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Không có argument, tạo cấu hình mặc định
        print("Không có đối số. Chạy với --help để xem hướng dẫn.")
        success = False
    else:
        # Có argument, chạy phân tích
        success = main()
    
    sys.exit(0 if success else 1)
