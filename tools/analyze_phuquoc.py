import os
import sys
import argparse
from pathlib import Path
import logging

# Thiết lập logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Thêm đường dẫn gốc của dự án vào sys.path để có thể import src
PROJECT_ROOT_PATH = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT_PATH))

from src.data.intersection_analyzer import IntersectionAnalyzer

def analyze_phuquoc_network(net_file: str, output_file: str):
    """
    Hàm tiện ích để phân tích mạng lưới Phú Quốc và tạo cấu hình intersection.
    """
    logging.info(f"Bắt đầu phân tích mạng lưới Phú Quốc từ {net_file}")
    
    if not os.path.exists(net_file):
        logging.error(f"❌ Không tìm thấy file network: {net_file}")
        return None
    
    analyzer = IntersectionAnalyzer(net_file)
    config_data = analyzer.generate_intersection_config(output_file)
    
    if config_data:
        logging.info(f"✅ Đã tạo file cấu hình: {output_file}")
    else:
        logging.error(f"❌ Lỗi khi tạo cấu hình cho {output_file}")
    
    return config_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Phân tích mạng lưới Phú Quốc và tạo cấu hình intersection.')
    parser.add_argument('--net-file', type=str, default=str(PROJECT_ROOT_PATH / 'src' / 'PhuQuoc' / 'phuquoc.net.xml'),
                       help='Đường dẫn đến file .net.xml của mạng lưới Phú Quốc')
    parser.add_argument('--output-file', type=str, default=str(PROJECT_ROOT_PATH / 'src' / 'config' / 'intersection_config.json'),
                       help='Đường dẫn đến file output JSON cho cấu hình intersection')
    args = parser.parse_args()

    analyze_phuquoc_network(args.net_file, args.output_file)
