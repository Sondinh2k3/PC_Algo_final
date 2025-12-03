"""
Lớp Điều khiển Chu vi Phản hồi (Perimeter Control)

Tác giả: Sơn Đình and Đức Ngô

MODIFIED: Tách biệt logic của bộ giải (solver) sang module riêng,
thêm hàm chạy thử nghiệm (mock test), và hỗ trợ nhiều pha phụ.
"""

import time
import os
import sys
import logging
from typing import Dict, Optional
from dataclasses import dataclass

# Thêm project root vào sys.path để giải quyết vấn đề import
# sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from data.intersection_config_manager import IntersectionConfigManager
from algorithm.solver import solve_green_time_optimization

# === CONSTANTS ===
KP_H = 20
KI_H = 5
N_HAT = 280.0
CONTROL_INTERVAL_S = 90

@dataclass
class ControlStepResult:
    """Lưu trữ kết quả của một bước điều khiển."""
    n_current: float
    qg_new: float
    is_active: bool

class PerimeterController:
    """
    Lớp điều khiển chu vi phản hồi.
    """
    
    # Thiết lập các tham số cho bộ điều khiển PI
    def __init__(self, kp: float = KP_H, ki: float = KI_H, n_hat: float = N_HAT, 
                 config_file: str = "src/config/intersection_config.json", shared_dict: Optional[Dict] = None,
                 control_interval_s: int = CONTROL_INTERVAL_S):
        control_interval_h = control_interval_s / 3600.0
        self.kp = kp * control_interval_h
        self.ki = ki * control_interval_h
        self.n_hat = n_hat
        
        self.shared_dict = shared_dict
        self.is_active = False

        # Ngưỡng kích hoạt và hủy kích hoạt thuật toán
        self.activation_threshold = 0.85 * self.n_hat
        self.deactivation_threshold = 0.70 * self.n_hat

        # Đọc file cấu hình các nút giao cần điều khiển
        self.config_manager = IntersectionConfigManager(config_file)
        self.intersection_ids = self.config_manager.get_intersection_ids()
        
        # Lấy và lưu trữ thời gian đèn xanh ban đầu (chu kỳ cố định)
        self.initial_green_times = self.config_manager.get_initial_green_times()
        self.previous_green_times = self.initial_green_times.copy()
        
        # Share dict được sử dụng để chia sẻ trạng thái với các thành phần khác trong chương trình
        if self.shared_dict is not None:
            self.shared_dict['is_active'] = self.is_active
            self.shared_dict['green_times'] = self.initial_green_times

        logging.info("Bộ điều khiển chu vi đã được khởi tạo (hỗ trợ nhiều pha phụ).")
        logging.info(f"Ngưỡng kích hoạt: n(k) > {self.activation_threshold:.0f} xe")
        logging.info(f"Ngưỡng hủy: n(k) < {self.deactivation_threshold:.0f} xe")
        logging.info(f"Số intersection: {len(self.intersection_ids)}")

    def check_activation_status(self, n_k: float):
        if n_k > self.activation_threshold:
            if not self.is_active:
                logging.info(f"KÍCH HOẠT ĐIỀU KHIỂN CHU VI (n(k)={n_k:.0f} > {self.activation_threshold:.0f})")
                self.is_active = True
        elif n_k < self.deactivation_threshold:
            if self.is_active:
                logging.info(f"HỦY ĐIỀU KHIỂN CHU VI (n(k)={n_k:.0f} < {self.deactivation_threshold:.0f}). Khôi phục chu kỳ đèn cố định.")
                self.is_active = False
                # Khôi phục lại thời gian đèn xanh ban đầu
                self.previous_green_times = self.initial_green_times.copy()
                if self.shared_dict is not None:
                    self.shared_dict['green_times'] = self.initial_green_times
        
        if self.shared_dict is not None:
            self.shared_dict['is_active'] = self.is_active

    def calculate_target_inflow(self, n_k: float, n_k_minus_1: float, qg_k_minus_1: float) -> float:
        error = self.n_hat - n_k
        change_in_n = n_k - n_k_minus_1
        qg_k = qg_k_minus_1 - (self.kp / (CONTROL_INTERVAL_S / 3600.0)) * change_in_n + (self.ki / (CONTROL_INTERVAL_S / 3600.0)) * error
        logging.debug(f"Sai số: e(k) = {self.n_hat:.0f} - {n_k:.0f} = {error:.1f} xe")
        logging.debug(f"Thay đổi: Δn(k) = {n_k:.0f} - {n_k_minus_1:.0f} = {change_in_n:.1f} xe")
        logging.info(f"PI Output: qg(k) = {qg_k:.2f} xe/giờ")
        return max(0, qg_k)

    def distribute_inflow_to_green_times(self, target_inflow: float, live_queue_lengths: Optional[Dict] = None):
        result = solve_green_time_optimization(
            target_inflow=target_inflow,
            config_manager=self.config_manager,
            previous_green_times=self.previous_green_times,
            live_queue_lengths=live_queue_lengths
        )
        
        if result:
            logging.info("Thời gian đèn xanh mới:")
            total_inflow = 0
            new_green_times = {}

            for int_id in self.intersection_ids:
                phase_info = self.config_manager.get_phase_info(int_id)
                if not phase_info: continue

                G_p = result['variables'][f'G_{int_id}_p']
                new_green_times[int_id] = {'p': int(G_p), 's': []}

                inflow_p = (G_p * phase_info['p']['saturation_flow'] * phase_info['p']['turn_in_ratio'])
                total_inflow += inflow_p
                logging.info(f"  {int_id}: G_p={G_p:.0f}s, inflow={inflow_p:.1f} xe/chu kỳ")

                if 's' in phase_info:
                    for i, _ in enumerate(phase_info['s']):
                        G_s = result['variables'][f'G_{int_id}_s_{i}']
                        new_green_times[int_id]['s'].append(int(G_s))
                        logging.info(f"    └─ G_s{i}={G_s:.0f}s")
            
            self.previous_green_times = new_green_times
            if self.shared_dict is not None:
                self.shared_dict['green_times'] = new_green_times
                self.shared_dict['new_logic_available'] = True

            logging.info(f"Tổng lưu lượng dự kiến (từ các pha chính): {total_inflow:.2f} xe/chu kỳ")
        else:
            logging.warning("Không tìm được nghiệm tối ưu, giữ nguyên thời gian đèn xanh.")

    def run_simulation_step(self, n_current: float, n_previous: float, qg_previous: float, live_queue_lengths: Optional[Dict] = None) -> ControlStepResult:
        logging.info(f"{ '='*15} BƯỚC ĐIỀU KHIỂN {'='*15}")
        logging.info(f"Đo lường - Trạng thái hiện tại: n(k) = {n_current:.0f} xe")
        self.check_activation_status(n_current)

        if not self.is_active:
            logging.info("Mục tiêu đã đạt được. Bộ điều khiển không hoạt động.")
            return ControlStepResult(n_current=n_current, qg_new=qg_previous, is_active=False)

        logging.info("Tính toán lưu lượng mục tiêu qg")
        qg_new = self.calculate_target_inflow(n_k=n_current, n_k_minus_1=n_previous, qg_k_minus_1=qg_previous)

        logging.info("Phân bổ thành thời gian đèn xanh")
        self.distribute_inflow_to_green_times(qg_new, live_queue_lengths)
        
        return ControlStepResult(n_current=n_current, qg_new=qg_new, is_active=True)


