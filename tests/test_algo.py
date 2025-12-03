import time
import os
import sys
import logging

# Thêm project root vào sys.path để có thể import từ src
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(PROJECT_ROOT)

from src.algorithm.algo import PerimeterController, N_HAT, CONTROL_INTERVAL_S

def run_perimeter_control_mock_test():
    print("🚦 BẮT ĐẦU MÔ PHỎNG THỬ NGHIỆM (MOCK TEST)")
    print("="*70)
    
    try:
        # Chạy từ thư mục gốc của dự án, nên đường dẫn tới config cần là "src/...."
        controller = PerimeterController(config_file="src/config/intersection_config.json")
    except Exception as e:
        print(f"\n[LỖI] Không thể khởi tạo bộ điều khiển: {e}")
        print("Vui lòng kiểm tra file 'src/config/intersection_config.json' và đảm bảo bạn đang chạy từ thư mục gốc của dự án.")
        return
    
    simulation_data = [
        {'step': 1, 'n_k': 80, 'description': 'Giao thông bình thường'},
        {'step': 2, 'n_k': 90, 'description': 'Lưu lượng tăng, gần ngưỡng'},
        {'step': 3, 'n_k': 100, 'description': 'Vượt ngưỡng - Kích hoạt'},
        {'step': 4, 'n_k': 110, 'description': 'Tắc nghẽn'},
        {'step': 5, 'n_k': 105, 'description': 'Bắt đầu cải thiện'},
        {'step': 6, 'n_k': 95, 'description': 'Tiếp tục giảm'},
        {'step': 7, 'n_k': 80, 'description': 'Dưới ngưỡng - Hủy kích hoạt'},
        {'step': 8, 'n_k': 75, 'description': 'Trở lại bình thường'},
    ]
    
    n_previous = 80.0
    qg_previous = 250.0
    
    print(f"\n THÔNG TIN MÔ PHỎNG:")
    print(f"   • Ngưỡng mục tiêu n̂: {N_HAT} xe")
    print(f"   • Khoảng điều khiển: {CONTROL_INTERVAL_S}s")
    print(f"   • Số bước mô phỏng: {len(simulation_data)} bước")
    print("\n" + "="*70)
    
    for data in simulation_data:
        step = data['step']
        n_current = data['n_k']
        description = data['description']
        
        print(f"\n CHU KỲ {step}: {description}")
        
        result = controller.run_simulation_step(
            n_current, n_previous, qg_previous
        )
        
        n_previous = result.n_current
        qg_previous = result.qg_new
        
        time.sleep(0.5)
    
    print(" KẾT THÚC MÔ PHỎNG THỬ NGHIỆM")
    print("="*70)

if __name__ == '__main__':
    run_perimeter_control_mock_test()
