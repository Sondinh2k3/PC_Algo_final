# Hệ thống Điều khiển Giao thông Thích ứng (Perimeter Control)

## 1. Giới thiệu

Dự án này là một hệ thống mô phỏng và điều khiển giao thông thông minh, được thiết kế để tối ưu hóa luồng xe trong một khu vực đô thị được xác định. Cốt lõi của hệ thống là thuật toán **Perimeter Control**, một phương pháp điều khiển thích ứng nhằm mục đích giữ cho mật độ phương tiện bên trong một "vành đai" (perimeter) không vượt quá một ngưỡng tới hạn, từ đó giảm thiểu tắc nghẽn và cải thiện hiệu suất mạng lưới.

Hệ thống sử dụng **SUMO (Simulation of Urban MObility)** làm môi trường mô phỏng, trong khi bộ điều khiển trung tâm được xây dựng bằng **Python** để thực thi thuật toán, xử lý dữ liệu và ra quyết định trong thời gian thực.

## 2. Kiến trúc hệ thống

Hệ thống bao gồm các thành phần chính sau:

- **SUMO Simulator**: Môi trường mô phỏng giao thông, cung cấp dữ liệu trực tiếp về vị trí, tốc độ, và hàng đợi của các phương tiện thông qua giao diện **Traci**.
- **Main Controller (`main.py`)**: Điểm khởi chạy và điều phối chính của hệ thống. Nó khởi tạo mô phỏng, quản lý các luồng dữ liệu và điều khiển.
- **Perimeter Control Algorithm (`src/algorithm/algo.py`)**: Lõi xử lý trung tâm, chứa logic của bộ điều khiển PI (Proportional-Integral) để tính toán tổng thời gian xanh cần thiết cho các luồng vào khu vực dựa trên sai số giữa mật độ xe hiện tại và mật độ mục tiêu.
- **Green Time Solver (`src/algorithm/solver.py`)**: Một bộ giải tối ưu hóa (sử dụng Google OR-Tools) để phân bổ tổng thời gian xanh (tính toán bởi `algo.py`) cho các pha đèn tại từng nút giao một cách công bằng và hiệu quả, dựa trên chiều dài hàng đợi.
- **Configuration Files (`src/config/`)**: Các file cấu hình YAML và JSON cho phép tùy chỉnh linh hoạt mọi khía cạnh của hệ thống, từ kịch bản mô phỏng, tham số thuật toán đến cấu hình nút giao.
- **Data Collectors & Tools**: Các module thu thập dữ liệu ra database (MySQL) và các công cụ dòng lệnh để hỗ trợ phân tích, trực quan hóa kết quả.

## 3. Công nghệ sử dụng

- **Ngôn ngữ lập trình**: Python 3.8+
- **Mô phỏng**: Eclipse SUMO
- **Giao tiếp SUMO**: Traci, Libsumo
- **Tối ưu hóa**: Google OR-Tools
- **Xử lý dữ liệu**: Pandas, NumPy
- **Cấu hình**: PyYAML
- **Database**: MySQL (qua Docker), Neo4j (tùy chọn)
- **Containerization**: Docker, Docker Compose

## 4. Cấu trúc thư mục

```
PC_Algo_v2/
├── docker-compose.yml        # Cấu hình Docker cho database MySQL.
├── requirements.txt          # Các thư viện Python cần thiết.
├── README.md                 # File README tổng quan.
├── Hương_dan_su_dung.md     # Hướng dẫn chi tiết cách cài đặt và sử dụng.
├── output/                   # Thư mục mặc định chứa kết quả mô phỏng.
├── src/                      # Mã nguồn chính của dự án.
│   ├── main.py               # Điểm khởi chạy chính.
│   ├── sumosim.py            # Lớp trừu tượng hóa việc tương tác với SUMO.
│   ├── algorithm/            # Lõi thuật toán điều khiển.
│   ├── config/               # Các file cấu hình hệ thống.
│   ├── data/                 # Các module xử lý, thu thập dữ liệu.
│   └── network_test/         # Mạng lưới SUMO dùng cho thử nghiệm.
└── tools/                    # Các script công cụ hỗ trợ.
```

## 5. Bắt đầu nhanh

1.  **Cài đặt**: Tham khảo file `HƯỚNG_DẪN_SỬ_DỤNG.md` để biết các bước cài đặt chi tiết.
2.  **Cấu hình**: Chỉnh sửa các file trong `src/config/` để phù hợp với kịch bản mô phỏng của bạn.
3.  **Chạy**: Thực thi lệnh `python src/main.py` để bắt đầu.

Để biết thêm chi tiết về cách cài đặt, cấu hình và vận hành hệ thống, vui lòng xem file **[HƯỚNG_DẪN_SỬ_DỤNG.md](HƯỚNG_DẪN_SỬ_DỤNG.md)**.

## 6. Generate kịch bản route:

python 'C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py' -c grid.conf.xml

# Test WSL