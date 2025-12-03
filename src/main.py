# -*- coding: utf-8 -*-

"""
Chương trình chính để khởi chạy và điều khiển mô phỏng giao thông bằng SUMO.

Kịch bản hoạt động:
1. Khởi tạo mô phỏng SUMO và các trình quản lý cấu hình.
2. Chạy một luồng (thread) riêng để cập nhật trạng thái đèn giao thông.
3. Vòng lặp mô phỏng chính thực hiện 3 bước theo chu kỳ:
    - BƯỚC 1: Thu thập dữ liệu thô (số xe, độ dài hàng đợi) từ các detector trong SUMO.
    - BƯỚC 2: Tổng hợp dữ liệu thô thành các giá trị trung bình.
    - BƯỚC 3: Chạy thuật toán điều khiển vành đai (Perimeter Control) và bộ giải (Solver)
              để tính toán thời gian xanh mới cho các đèn tín hiệu.
4. Luồng điều khiển đèn sẽ nhận thời gian xanh mới và cập nhật vào mô phỏng.
5. Mô phỏng kết thúc khi hết thời gian hoặc không còn xe.
"""

import traci
import yaml
import threading
import time
import os
import sys
import logging
from multiprocessing import Manager
from typing import Dict, Any, List

# Import các thành phần cần thiết từ các module khác trong dự án
from sumosim import SumoSim
from data.intersection_config_manager import IntersectionConfigManager
from data.detector_config_manager import DetectorConfigManager
from algorithm.algo import (
    PerimeterController, 
    KP_H, 
    KI_H, 
    N_HAT, 
    CONTROL_INTERVAL_S
)
from visualizer import RealTimePlotter

# --- CẤU HÌNH LOGGING ---
# Thiết lập hệ thống ghi log để theo dõi hoạt động của chương trình.
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# =============================================================================
# CÁC HÀM TẢI CẤU HÌNH
# =============================================================================

def load_yaml_config(config_path: str) -> Dict[str, Any]:
    """
    Tải và phân tích một file cấu hình YAML.

    Args:
        config_path: Đường dẫn đến file YAML.

    Returns:
        Một dictionary chứa nội dung của file cấu hình.
    
    Raises:
        ValueError: Nếu file không tồn tại hoặc rỗng.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"File cấu hình không tồn tại: {config_path}")
    with open(config_path, "r", encoding='utf-8') as f:
        config = yaml.safe_load(f)
        if config is not None:
            return config
        else:
            raise ValueError(f"File cấu hình rỗng hoặc không hợp lệ: {config_path}")

# =============================================================================
# LUỒNG ĐIỀU KHIỂN ĐÈN GIAO THÔNG
# =============================================================================

def update_traffic_light_logic(tl_id: str, new_times: Dict[str, Any], phase_info: Dict[str, Any]):
    """
    Cập nhật logic (thời gian xanh) cho một đèn giao thông cụ thể.

    Args:
        tl_id: ID của đèn giao thông trong SUMO.
        new_times: Dictionary chứa thời gian xanh mới cho pha chính ('p') và các pha phụ ('s').
        phase_info: Thông tin về các chỉ số pha chính và phụ.
    """
    try:
        # Lấy định nghĩa đầy đủ của đèn (bao gồm các pha)
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]

        # Cập nhật thời gian xanh cho các pha chính
        main_phases = phase_info.get('p', {}).get('phase_indices', [])
        for phase_index in main_phases:
            if 0 <= phase_index < len(logic.phases):
                logic.phases[phase_index].duration = new_times['p']

        # Cập nhật thời gian xanh cho các pha phụ
        secondary_phases = [s_phase['phase_indices'][0] for s_phase in phase_info.get('s', []) if s_phase.get('phase_indices')]
        for i, phase_index in enumerate(secondary_phases):
            if 0 <= phase_index < len(logic.phases) and i < len(new_times['s']):
                logic.phases[phase_index].duration = new_times['s'][i]
        
        # Áp dụng logic mới vào mô phỏng
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)

    except traci.TraCIException as e:
        logging.error(f"Lỗi Traci khi cập nhật TLS {tl_id}: {e}")

# Dictionary để lưu các kế hoạch đèn đang chờ được áp dụng
# Định dạng: { "intersection_id": new_green_times_for_that_intersection }
pending_logic_updates = {}

def prepare_logic_update(shared_dict: Dict):
    """
    Lấy dữ liệu thời gian xanh mới từ shared_dict và lưu vào danh sách chờ
    'pending_logic_updates' để được áp dụng vào chu kỳ đèn tiếp theo.
    """
    global pending_logic_updates
    try:
        if shared_dict.get('is_active', False) and 'green_times' in shared_dict:
            green_times = shared_dict.get('green_times')
            if green_times:
                # Chỉ lưu lại kế hoạch, không áp dụng ngay
                pending_logic_updates.update(green_times)
                logging.info(f"Đã nhận và lên lịch cập nhật cho các giao lộ: {list(green_times.keys())}")
                # Xóa dữ liệu đã xử lý khỏi shared_dict để tránh lặp lại
                del shared_dict['green_times']
    except Exception as e:
        logging.error(f"Lỗi khi chuẩn bị cập nhật logic đèn: {e}", exc_info=True)

def apply_pending_updates_on_cycle_start(config_manager: IntersectionConfigManager, sim_step: float):
    """
    Kiểm tra các giao lộ có thay đổi đang chờ và áp dụng chúng khi chu kỳ đèn
    hiện tại sắp kết thúc. Hàm này phải được gọi mỗi bước mô phỏng.
    """
    global pending_logic_updates
    if not pending_logic_updates:
        return  # Không có gì để làm

    # Duyệt qua một bản sao của các keys để có thể xóa phần tử trong vòng lặp
    for int_id in list(pending_logic_updates.keys()):
        tl_id = config_manager.get_traffic_light_id(int_id)
        if not tl_id:
            continue

        try:
            # Lấy thông tin chương trình đèn hiện tại
            program_logics = traci.trafficlight.getAllProgramLogics(tl_id)
            current_program_id = traci.trafficlight.getProgram(tl_id)
            current_logic = next((p for p in program_logics if p.programID == current_program_id), None)
            if not current_logic:
                continue

            current_phase_index = traci.trafficlight.getPhase(tl_id)
            num_phases = len(current_logic.phases)
            time_to_next_switch = traci.trafficlight.getNextSwitch(tl_id) - traci.simulation.getTime()

            # Điều kiện kích hoạt: đang ở pha cuối cùng VÀ thời gian đến lần chuyển tiếp theo
            # rất ngắn (bằng 1 bước mô phỏng), nghĩa là chu kỳ sắp reset.
            is_last_phase = (current_phase_index == num_phases - 1)
            is_about_to_reset = (time_to_next_switch <= sim_step)

            if is_last_phase and is_about_to_reset:
                new_green_times = pending_logic_updates[int_id]
                phase_info = config_manager.get_phase_info(int_id)

                if phase_info:
                    logging.info(f"Áp dụng logic mới cho giao lộ {int_id} (TL: {tl_id}) vào đầu chu kỳ tiếp theo.")
                    update_traffic_light_logic(tl_id, new_green_times, phase_info)
                    # Xóa khỏi danh sách chờ sau khi đã áp dụng
                    del pending_logic_updates[int_id]
                else:
                    logging.warning(f"Bỏ qua áp dụng cho {int_id} do thiếu phase_info.")

        except traci.TraCIException as e:
            logging.error(f"Lỗi TraCI khi kiểm tra giao lộ {int_id}: {e}")
            # Có thể xóa key nếu giao lộ không còn tồn tại trong mô phỏng
            if "does not exist" in str(e):
                del pending_logic_updates[int_id]
        except Exception as e:
            logging.error(f"Lỗi không xác định khi áp dụng cập nhật cho {int_id}: {e}", exc_info=True)


# =============================================================================
# CÁC HÀM HỖ TRỢ VÒNG LẶP MÔ PHỎNG
# =============================================================================

def get_sum_from_traci_detectors(detector_ids: List[str]) -> int:
    """
    Lấy số lượng phương tiện (tích lũy) dựa trên độ chiếm dụng theo không gian.
    An toàn trước các lỗi Traci và lỗi chia cho 0.
    """
    try:
        total_accumulation = 0
        for det_id in detector_ids:
            space_occupancy = traci.lanearea.getLastIntervalOccupancy(det_id)
            road_length = 80.00
            average_length_of_vehicles = 3
            num_lane = 1
            # Tính toán tích lũy tại một detector:
            accumulation = road_length * (num_lane / (100 * average_length_of_vehicles)) * space_occupancy
            total_accumulation += accumulation
        return total_accumulation # Trả về 0 nếu không có xe, tốc độ trung bình hoặc interval bằng 0

    except traci.TraCIException as e:
        logging.warning(f"Lỗi TraCI trong get_sum_from_traci_detectors: {e}")
        return 0


def initialize_queue_samples(solver_detectors: Dict) -> Dict:
    """Khởi tạo cấu trúc dữ liệu để lưu trữ các mẫu hàng đợi."""
    queue_samples = {}
    for int_id, int_details in solver_detectors.items():
        num_secondary_phases = len(int_details.get('phases', {}).get('s', []))
        queue_samples[int_id] = {
            'p': [],
            's': [[] for _ in range(num_secondary_phases)]
        }
    return queue_samples

def clear_samples(n_samples: List, queue_samples: Dict):
    """Xóa tất cả các mẫu đã thu thập để chuẩn bị cho chu kỳ tổng hợp tiếp theo."""
    n_samples.clear()
    for int_id in queue_samples:
        queue_samples[int_id]['p'].clear()
        for s_phase_samples in queue_samples[int_id]['s']:
            s_phase_samples.clear()

# =============================================================================
# HÀM CHẠY MÔ PHỎNG CHÍNH
# =============================================================================

def run_sumo_simulation():
    """Hàm chính để khởi tạo và chạy toàn bộ kịch bản mô phỏng SUMO."""
    try:
        # --- 1. KHỞI TẠO --- 
        logging.info("Bắt đầu quá trình khởi tạo mô phỏng...")

        # Xác định các đường dẫn file cấu hình
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) 
        sim_config_path = os.path.join(project_root, 'src', 'config', 'simulation.yml')
        detector_config_path = os.path.join(project_root, 'src', 'config', 'detector_config.json')
        intersection_config_path = os.path.join(project_root, 'src', 'config', 'intersection_config.json')

        # Tải các file cấu hình
        sim_config = load_yaml_config(sim_config_path).get('config', {})
        detector_config_mgr = DetectorConfigManager(detector_config_path)
        intersection_config_mgr = IntersectionConfigManager(intersection_config_path)

        # Lấy các tham số mô phỏng
        sampling_interval_s = sim_config.get('sampling_interval_s', 10)
        aggregation_interval_s = sim_config.get('aggregation_interval_s', 50)
        total_simulation_time = sim_config.get('total_simulation_time', 3600)

        # Lấy ID của các detector cần thiết
        algorithm_detector_ids = detector_config_mgr.get_algorithm_input_detectors()
        solver_detectors = detector_config_mgr.get_solver_input_detectors()
        flow_algorithm_detector = detector_config_mgr.get_mfd_input_flow_detectors()
        logging.info(f"Tìm thấy {len(algorithm_detector_ids)} detectors cho thuật toán vành đai.")
        logging.info(f"Tìm thấy {len(solver_detectors)} giao lộ cho bộ giải cục bộ.")
        logging.info(f"Tìm thấy {len(flow_algorithm_detector)} detectors cho flow đầu vào thuật toán.")

        # --- 2. THIẾT LẬP MÔI TRƯỜNG ĐA LUỒNG VÀ SUMO ---
        with Manager() as manager:
            shared_dict = manager.dict() # Dict để giao tiếp giữa luồng chính và thuật toán

            # Khởi động SUMO
            sumo_sim = SumoSim(sim_config)
            output_dir = os.path.join(project_root, "output")
            os.makedirs(output_dir, exist_ok=True)
            output_files = {"tripinfo": os.path.join(output_dir, "tripinfo.xml")}
            sumo_sim.start(output_files=output_files)

            # Khởi tạo bộ điều khiển chính
            controller = PerimeterController(
                kp=KP_H, ki=KI_H, n_hat=N_HAT, 
                config_file=intersection_config_path,
                shared_dict=shared_dict
            )

            # Khởi tạo bộ vẽ đồ thị thời gian thực
            plotter = RealTimePlotter(set_point=controller.n_hat)

            # --- 3. CHUẨN BỊ CHO VÒNG LẶP CHÍNH ---
            n_previous = 0
            qg_previous = 0
            
            # Biến lưu trữ dữ liệu thu thập được
            n_samples = []
            queue_samples = initialize_queue_samples(solver_detectors)
            
            # Biến lưu trữ dữ liệu đã được tổng hợp
            latest_aggregated_n = 0
            latest_aggregated_queue_lengths = {}

            # Lấy giá trị ban đầu và các thông số mô phỏng
            sim_step = traci.simulation.getDeltaT()
            sumo_sim.step()
            n_previous = get_sum_from_traci_detectors(algorithm_detector_ids)
            latest_aggregated_n = n_previous

            # Thiết lập các mốc thời gian cho các hành động
            next_sampling_time = 0
            next_aggregation_time = aggregation_interval_s
            next_control_time = CONTROL_INTERVAL_S
            next_log_time = 10

            logging.info("Khởi tạo hoàn tất. Bắt đầu vòng lặp mô phỏng chính.")

            # --- 4. VÒNG LẶP MÔ PHỎNG CHÍNH ---
            while traci.simulation.getMinExpectedNumber() > 0:
                sumo_sim.step()
                current_time = traci.simulation.getTime()

                # --- BƯỚC 1: THU THẬP DỮ LIỆU MẪU ---
                if current_time >= next_sampling_time:
                    n_samples.append(get_sum_from_traci_detectors(algorithm_detector_ids))

                    for int_id, details in solver_detectors.items():
                        phases = details.get('phases', {})
                        # Hàng đợi pha chính
                        p_detectors = phases.get('p', {}).get('queue_detectors', [])
                        queue_samples[int_id]['p'].append(get_sum_from_traci_detectors(p_detectors))
                        # Hàng đợi các pha phụ
                        for i, s_phase in enumerate(phases.get('s', [])):
                            s_detectors = s_phase.get('queue_detectors', [])
                            if i < len(queue_samples[int_id]['s']):
                                queue_samples[int_id]['s'][i].append(get_sum_from_traci_detectors(s_detectors))
                    
                    next_sampling_time += sampling_interval_s

                # --- BƯỚC 2: TỔNG HỢP DỮ LIỆU ---
                if current_time >= next_aggregation_time:
                    logging.info(f"--- Tổng hợp dữ liệu tại t={current_time:.1f}s ---")
                    
                    if n_samples:
                        latest_aggregated_n = sum(n_samples) / len(n_samples)
                        # Cập nhật đồ thị với giá trị n(k) mới
                        plotter.update_plot(current_time, latest_aggregated_n)

                    for int_id, data in queue_samples.items():
                        avg_p = sum(data['p']) / len(data['p']) if data['p'] else 0
                        avg_s = [sum(s) / len(s) if s else 0 for s in data['s']]
                        latest_aggregated_queue_lengths[int_id] = {'p': avg_p, 's': avg_s}

                    logging.info(f"n(k) mới={latest_aggregated_n:.2f}. Xóa {len(n_samples)} mẫu.")
                    clear_samples(n_samples, queue_samples)
                    next_aggregation_time += aggregation_interval_s

                # --- BƯỚC 3: CHẠY THUẬT TOÁN ĐIỀU KHIỂN ---
                if current_time >= next_control_time:
                    logging.info(f"--- Chạy điều khiển tại t={current_time:.1f}s ---")
                    
                    # Chạy thuật toán để tính toán thời gian xanh mới và lưu vào shared_dict
                    result = controller.run_simulation_step(
                        latest_aggregated_n, n_previous, qg_previous, latest_aggregated_queue_lengths
                    )
                    
                    # Chuẩn bị/lên lịch cập nhật thay vì áp dụng ngay
                    prepare_logic_update(shared_dict)

                    # Cập nhật các biến trạng thái cho chu kỳ tiếp theo
                    qg_previous = result.qg_new
                    n_previous = latest_aggregated_n
                    next_control_time += CONTROL_INTERVAL_S
                
                # --- BƯỚC 4: ÁP DỤNG CÁC CẬP NHẬT ĐANG CHỜ (MỖI BƯỚC) ---
                apply_pending_updates_on_cycle_start(intersection_config_mgr, sim_step)

                # Ghi log tiến độ và kiểm tra điều kiện dừng
                if current_time >= next_log_time:
                    logging.info(f"Thời gian: {current_time:.0f}s / {total_simulation_time}s")
                    next_log_time += 10

                if current_time >= total_simulation_time:
                    logging.info(f"Đạt thời gian mô phỏng tối đa. Dừng lại.")
                    break

    except (traci.TraCIException, traci.FatalTraCIError) as e:
        logging.warning(f"Kết nối Traci bị đóng hoặc mô phỏng kết thúc sớm: {e}")
    except Exception as e:
        logging.error(f"Lỗi không mong muốn trong quá trình chạy mô phỏng: {e}", exc_info=True)
    finally:
        # --- 5. DỌN DẸP VÀ KẾT THÚC ---
        logging.info("Đóng mô phỏng.")
        if 'sumo_sim' in locals() and sumo_sim.is_running():
            sumo_sim.close()
            logging.info(f"Mô phỏng kết thúc. Tổng số bước: {sumo_sim.get_step_counts()}")

        # Giữ đồ thị hiển thị cho đến khi người dùng đóng nó
        if 'plotter' in locals() and plotter is not None:
            logging.info("Đóng cửa sổ đồ thị để kết thúc hoàn toàn chương trình.")
            plotter.close()

if __name__ == "__main__":
    # Mặc định, chương trình sẽ chạy mô phỏng với SUMO.
    # Tùy chọn chạy 'mock' (thử nghiệm giả lập) hiện không được sử dụng.
    if len(sys.argv) > 1 and sys.argv[1] == 'mock':
        logging.warning("Chức năng 'mock' test hiện không có sẵn.")
    else:
        run_sumo_simulation()
