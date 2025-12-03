"""
Intersection Config Manager - Quản lý cấu hình nút giao từ file JSON
Đọc và cung cấp dữ liệu cho bài toán tối ưu hóa.
MODIFIED: Hỗ trợ cấu trúc pha linh hoạt (1 pha chính, nhiều pha phụ).
"""

import json
import os
import logging
from typing import Dict, List, Optional, Any

class IntersectionConfigManager:
    """
    Quản lý cấu hình intersection từ file JSON với cấu trúc pha linh hoạt.
    """
    
    def __init__(self, config_file: str = "src/config/intersection_config.json"):
        """
        Khởi tạo config manager.
        
        Args:
            config_file: Đường dẫn đến file cấu hình JSON.
        """
        self.config_file = config_file
        self.config_data = {}
        self.load_config()
    
    def load_config(self) -> bool:
        """
        Load cấu hình từ file JSON.
        """
        try:
            if not os.path.exists(self.config_file):
                logging.error(f"Không tìm thấy file cấu hình tại '{self.config_file}'")
                return False
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config_data = json.load(f)
            
            logging.info(f"Đã load cấu hình từ: {self.config_file}")
            return True
            
        except Exception as e:
            logging.error(f"Lỗi khi load cấu hình: {e}", exc_info=True)
            return False

    def save_config(self, output_file: Optional[str] = None):
        """
        Lưu cấu hình vào file JSON.
        """
        if output_file is None:
            output_file = self.config_file
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(self.config_data, f, indent=2, ensure_ascii=False)
            logging.info(f"Đã lưu cấu hình vào: {output_file}")
        except Exception as e:
            logging.error(f"Lỗi khi lưu cấu hình: {e}", exc_info=True)
    
    def get_intersection_ids(self) -> List[str]:
        """
        Lấy danh sách ID của các intersection được định nghĩa trong 'optimization_parameters'.
        """
        return self.config_data.get('optimization_parameters', {}).get('intersection_ids', [])
    
    def get_global_params(self) -> Dict[str, Any]:
        """
        Lấy các tham số toàn cục cho bài toán tối ưu hóa.
        """
        params = self.config_data.get('optimization_parameters', {})
        return {
            'theta_1': params.get('theta_1', 1.0),
            'theta_2': params.get('theta_2', 0.5),
            'default_cycle_length': params.get('default_cycle_length', 90),
            'min_green_time': params.get('min_green_time', 15),
            'max_green_time': params.get('max_green_time', 75),
            'max_change': params.get('max_change', 5)
        }

    def get_intersection_data(self, intersection_id: str) -> Optional[Dict]:
        """
        Lấy toàn bộ dữ liệu của một intersection cụ thể.
        """
        return self.config_data.get('optimization_parameters', {}).get('intersection_data', {}).get(intersection_id)

    def get_cycle_length(self, intersection_id: str) -> int:
        """
        Lấy chu kỳ đèn của intersection.
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('cycle_length', 90)
        return 90

    def get_traffic_light_id(self, intersection_id: str) -> Optional[str]:
        """
        Lấy ID đèn giao thông của một intersection.
        """
        intersection_data = self.config_data.get('intersections', {}).get(intersection_id)
        if intersection_data:
            return intersection_data.get('traffic_light_id')
        return None

    def get_phase_info(self, intersection_id: str) -> Optional[Dict]:
        """
        Lấy thông tin về các pha (chính và phụ) của một intersection.
        
        Returns:
            Dict: Một dict chứa thông tin về pha chính ('p') và danh sách các pha phụ ('s').
                  Ví dụ: {'p': {...}, 's': [{...}, {...}]}
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('phases')
        return None

    def get_initial_green_times(self) -> Dict[str, Dict[str, Any]]:
        """
        Lấy thời gian đèn xanh ban đầu từ file cấu hình cho tất cả các intersection.
        """
        initial_green_times = {}
        intersection_ids = self.get_intersection_ids()

        for int_id in intersection_ids:
            tl_id = self.get_traffic_light_id(int_id)
            if not tl_id:
                logging.warning(f"Không tìm thấy traffic_light_id cho intersection {int_id}. Bỏ qua khởi tạo.")
                continue

            # Lấy định nghĩa các pha từ traffic_lights, không phải từ traci
            traffic_light_phases = self.config_data.get('traffic_lights', {}).get(tl_id, {}).get('phases', [])
            phase_info = self.get_phase_info(int_id)

            if not traffic_light_phases or not phase_info:
                logging.warning(f"Không tìm thấy thông tin pha cho đèn {tl_id} hoặc intersection {int_id}. Bỏ qua khởi tạo.")
                continue

            main_phase_duration = 0
            if 'p' in phase_info and 'phase_indices' in phase_info['p']:
                main_phase_index = phase_info['p']['phase_indices'][0]
                if 0 <= main_phase_index < len(traffic_light_phases):
                    main_phase_duration = int(traffic_light_phases[main_phase_index].get('duration', 0))

            secondary_phase_durations = []
            if 's' in phase_info:
                for s_phase_config in phase_info['s']:
                    if 'phase_indices' in s_phase_config:
                        s_phase_index = s_phase_config['phase_indices'][0]
                        if 0 <= s_phase_index < len(traffic_light_phases):
                            secondary_phase_durations.append(int(traffic_light_phases[s_phase_index].get('duration', 0)))

            initial_green_times[int_id] = {
                'p': main_phase_duration,
                's': secondary_phase_durations
            }
        return initial_green_times
