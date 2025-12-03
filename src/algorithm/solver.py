"""
Module giải bài toán tối ưu hóa phân bổ thời gian đèn xanh.
Sử dụng PySCIPOpt để giải bài toán MIQP (Mixed-Integer Quadratic Programming) phi tuyến.
MODIFIED: Hỗ trợ cấu trúc pha linh hoạt (1 pha chính, nhiều pha phụ).
"""

from pyscipopt import Model, quicksum
from pyscipopt.recipes.nonlinear import set_nonlinear_objective
from typing import Dict, Optional

from algorithm.common import SolverStatus
from data.intersection_config_manager import IntersectionConfigManager

def solve_green_time_optimization(
    target_inflow: float, # qg: veh/h duoc tinh toan boi bo dieu khien PC
    config_manager: IntersectionConfigManager, # cac du lieu lien quan den nut giao
    previous_green_times: Dict, # Thong tin thoi gian xanh cua chu ky truoc
    live_queue_lengths: Optional[Dict] = None
) -> Optional[Dict]:
    """
    Giải bài toán tối ưu hóa để phân bổ lưu lượng mục tiêu (qg) thành thời gian đèn xanh
    hỗ trợ nhiều pha phụ và sử dụng hàng đợi trực tiếp.

    Args:
        target_inflow: Lưu lượng vào mục tiêu qg(k) [xe/giờ].
        config_manager: Đối tượng quản lý cấu hình intersection.
        previous_green_times: Dict chứa thời gian xanh của chu kỳ trước.
        live_queue_lengths: Dict chứa độ dài hàng đợi thực tế từ mô phỏng.

    Returns:
        Một dict chứa kết quả nếu tìm thấy nghiệm tối ưu, ngược lại trả về None.
    """
    global_params = config_manager.get_global_params()
    intersection_ids = config_manager.get_intersection_ids()

    cycle_length = global_params.get('default_cycle_length', 90)
    qg_prime = target_inflow * cycle_length / 3600.0
    
    print(f"🔧 Giải bài toán MIQP với mục tiêu qg = {target_inflow:.2f} [xe/giờ]")
    print(f"   (Tương đương {qg_prime:.2f} [xe / chu kỳ đèn {cycle_length}s])")

    model = Model("MIQP_PerimeterControl_MultiPhase")

    theta_1 = global_params.get('theta_1', 1.0)
    theta_2 = global_params.get('theta_2', 0.5)
    min_green = global_params.get('min_green_time', 15)
    max_change = global_params.get('max_change', 10)

    G_vars = {}
    for int_id in intersection_ids:
        G_vars[int_id] = {'p': None, 's': {}}
        # max_green for each intersection can be different based on its cycle length
        int_max_green = config_manager.get_cycle_length(int_id) - min_green
        phase_info = config_manager.get_phase_info(int_id)

        # Tạo biến cho pha chính (primary)
        G_p = model.addVar(f'G_{int_id}_p', vtype='INTEGER', lb=min_green, ub=int_max_green)
        G_vars[int_id]['p'] = G_p

        # Tạo biến cho các pha phụ (secondary)
        if phase_info and 's' in phase_info:
            for i, _ in enumerate(phase_info['s']):
                G_s = model.addVar(f'G_{int_id}_s_{i}', vtype='INTEGER', lb=min_green, ub=int_max_green)
                G_vars[int_id]['s'][i] = G_s

    # Xác định độ dài hàng đợi sẽ sử dụng (trực tiếp hoặc từ config)
    queue_lengths_to_use = {}
    for int_id in intersection_ids:
        phase_info = config_manager.get_phase_info(int_id)
        queue_lengths_to_use[int_id] = {'p': 0, 's': []}
        
        # Pha chính
        if live_queue_lengths and int_id in live_queue_lengths:
            queue_lengths_to_use[int_id]['p'] = live_queue_lengths[int_id]['p']
        else:
            queue_lengths_to_use[int_id]['p'] = phase_info['p']['queue_length']
            
        # Pha phụ
        if 's' in phase_info:
            for i, phase in enumerate(phase_info['s']):
                if live_queue_lengths and int_id in live_queue_lengths and i < len(live_queue_lengths[int_id]['s']):
                    queue_lengths_to_use[int_id]['s'].append(live_queue_lengths[int_id]['s'][i])
                else:
                    queue_lengths_to_use[int_id]['s'].append(phase['queue_length'])

    # Thêm ràng buộc
    for int_id in intersection_ids:
        phase_info = config_manager.get_phase_info(int_id)
        current_cycle = config_manager.get_cycle_length(int_id)

        # Ràng buộc 1: Tổng thời gian xanh = chu kỳ đèn
        secondary_phases_sum = quicksum(G_vars[int_id]['s'][i] for i in G_vars[int_id]['s'])
        model.addCons(G_vars[int_id]['p'] + secondary_phases_sum == current_cycle, f"cons_cycle_{int_id}")

        # Ràng buộc 2: Giới hạn thay đổi so với chu kỳ trước
        prev_p = previous_green_times[int_id]['p']
        model.addCons(G_vars[int_id]['p'] >= prev_p - max_change, f"cons_G_p_min_{int_id}")
        model.addCons(G_vars[int_id]['p'] <= prev_p + max_change, f"cons_G_p_max_{int_id}")
        print(f"  Intersection {int_id} - Main Phase (p): Previous={prev_p}, Bounds=[{prev_p - max_change}, {prev_p + max_change}], Var_Bounds=[{min_green}, {int_max_green}]")

        if phase_info and 's' in phase_info:
            for i, _ in enumerate(phase_info['s']):
                prev_s = previous_green_times[int_id]['s'][i]
                model.addCons(G_vars[int_id]['s'][i] >= prev_s - max_change, f"cons_G_s{i}_min_{int_id}")
                model.addCons(G_vars[int_id]['s'][i] <= prev_s + max_change, f"cons_G_s{i}_max_{int_id}")
                print(f"  Intersection {int_id} - Secondary Phase (s{i}): Previous={prev_s}, Bounds=[{prev_s - max_change}, {prev_s + max_change}], Var_Bounds=[{min_green}, {int_max_green}]")

    # Xây dựng hàm mục tiêu phi tuyến
    # Thành phần 1: Tối thiểu hóa độ lệch so với lưu lượng mục tiêu (chỉ tính trên pha chính)
    inflow_expr = quicksum(
        G_vars[int_id]['p'] * config_manager.get_phase_info(int_id)['p']['saturation_flow'] * config_manager.get_phase_info(int_id)['p']['turn_in_ratio']
        for int_id in intersection_ids
    )
    deviation = inflow_expr - qg_prime
    first_component = theta_1 * (deviation**2)

    # Thành phần 2: Tối đa hóa việc sử dụng đèn xanh (tối thiểu hóa lãng phí)
    utilization_expr = quicksum(
        # Lãng phí của pha chính
        (1 - (G_vars[int_id]['p'] * config_manager.get_phase_info(int_id)['p']['saturation_flow']) / 
            (queue_lengths_to_use[int_id]['p'] + 1))**2 +
        # Tổng lãng phí của các pha phụ
        quicksum(
            (1 - (G_vars[int_id]['s'][i] * phase['saturation_flow']) / (queue_lengths_to_use[int_id]['s'][i] + 1))**2
            for i, phase in enumerate(config_manager.get_phase_info(int_id)['s'])
        )
        for int_id in intersection_ids
    )
    second_component = theta_2 * utilization_expr

    set_nonlinear_objective(model, first_component + second_component, "minimize")

    model.hideOutput()
    model.optimize()

    if model.getStatus() == "optimal":
        print(f"  Tìm được nghiệm tối ưu: {model.getStatus()}")
        result = {
            'status': SolverStatus.OPTIMAL,
            'objective_value': model.getObjVal(),
            'variables': {var.name: model.getVal(var) for var in model.getVars()}
        }
        return result
    else:
        print(f"  Không tìm được nghiệm: {model.getStatus()}")
        return None