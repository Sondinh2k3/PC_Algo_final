# Hướng dẫn Cài đặt và Sử dụng Hệ thống

## 1. Yêu cầu Môi trường

Trước khi bắt đầu, hãy đảm bảo bạn đã cài đặt các phần mềm sau:

- **Python**: Phiên bản 3.8 trở lên.
- **Eclipse SUMO**: Công cụ mô phỏng giao thông. [Tải tại đây](https://www.eclipse.org/sumo/).
- **Docker và Docker Compose**: Để chạy các dịch vụ phụ trợ như database. [Tải tại đây](https://www.docker.com/products/docker-desktop/).
- **Git**: Để sao chép mã nguồn dự án.

## 2. Cài đặt

### Bước 1: Cấu hình biến môi trường cho SUMO

Sau khi cài đặt SUMO, bạn cần thêm đường dẫn của nó vào biến môi trường hệ thống để các script Python có thể gọi.

1.  Tạo một biến môi trường tên là `SUMO_HOME` và trỏ đến thư mục cài đặt gốc của SUMO.
    -   *Ví dụ trên Windows:* `C:\ Program Files (x88)\Eclipse\Sumo`
2.  Thêm thư mục `tools` của SUMO vào biến môi trường `Path`.
    -   *Ví dụ trên Windows:* `%SUMO_HOME%\tools`

### Bước 2: Tải mã nguồn

Mở terminal và clone repository về máy của bạn:

```bash
git clone <URL_CUA_REPOSITORY>
cd PC_Algo_v2
```

### Bước 3: Cài đặt các thư viện Python

Tạo một môi trường ảo (virtual environment) để tránh xung đột thư viện (khuyến khích) và cài đặt các gói cần thiết.

```bash
# Tạo môi trường ảo
python -m venv venv

# Kích hoạt môi trường ảo
# Trên Windows:
venv\Scripts\activate
# Trên macOS/Linux:
# source venv/bin/activate

# Cài đặt các thư viện từ file requirements.txt
pip install -r requirements.txt
```

### Bước 4: Khởi chạy dịch vụ phụ trợ (Database)

Dự án sử dụng MySQL để lưu trữ dữ liệu thu thập được. `docker-compose` sẽ giúp bạn khởi tạo container một cách dễ dàng.

```bash
docker-compose up -d
```

Lệnh này sẽ tải image MySQL và chạy một container ở chế độ nền. Dữ liệu sẽ được lưu trong một volume tên là `db_data`.

## 3. Cấu hình Mô phỏng

Toàn bộ cấu hình được quản lý trong thư mục `src/config`.

### `simulation.yml`

File này định nghĩa kịch bản SUMO sẽ được chạy.

-   `config_file`: Đường dẫn tới file `.sumocfg`.
-   `net_file`: Đường dẫn tới file mạng lưới `.net.xml`.
-   `route_file`: Đường dẫn tới file luồng xe `.rou.xml`.
-   `gui`: Đặt là `true` để hiển thị giao diện đồ họa của SUMO, `false` để chạy trong nền (nhanh hơn).
-   `total_simulation_time`: Tổng thời gian mô phỏng (tính bằng giây).

### `application.yml`

Chứa thông tin kết nối đến các cơ sở dữ liệu.

-   `mysql`: Cấu hình cho MySQL. Mật khẩu và port phải khớp với những gì bạn đã định nghĩa trong `docker-compose.yml`.

### `detector_config.json`

Định nghĩa các cảm biến (detectors) trong SUMO mà thuật toán sẽ sử dụng.

-   `algorithm_input_detectors`: Danh sách ID của các cảm biến `e2` (entry/exit detectors) dùng để đo tổng số xe (accumulation) trong khu vực.
-   `solver_input_detectors`: Danh sách ID của các cảm biến `e1` (lane area detectors) dùng để đo chiều dài hàng đợi tại các lối vào nút giao.

### `intersection_config.json`

File cấu hình quan trọng nhất, chứa thông tin chi tiết về các nút giao sẽ được điều khiển.

-   `intersections`: Một danh sách các đối tượng, mỗi đối tượng đại diện cho một nút giao (`id`) và ID của đèn tín hiệu tương ứng (`tl_id`).
-   `optimization_parameters`:
    -   `intersection_ids`: Các nút giao sẽ được bộ giải (`solver`) tối ưu hóa.
    -   `intersection_data`: Chứa các tham số kỹ thuật cho từng nút giao như `cycle_length` (thời gian chu kỳ đèn), `main_phases` (danh sách index các pha chính), `saturation_flows` (lưu lượng bão hòa),...

Bạn có thể tự động tạo một file `intersection_config.json` cơ bản từ file `.net.xml` bằng cách sử dụng công cụ đi kèm:

```bash
python tools/generate_intersection_config.py --net-file path/to/your/net.xml --output src/config/intersection_config.json
```

Sau đó, bạn cần tinh chỉnh lại file này để thêm các thông số như `saturation_flows`.

## 4. Chạy Hệ thống

Sau khi đã hoàn tất cài đặt và cấu hình, bạn có thể khởi chạy mô phỏng.

```bash
# Đảm bảo bạn đang ở trong thư mục gốc của dự án
python src/main.py
```

Nếu `gui: true` trong `simulation.yml`, cửa sổ SUMO GUI sẽ xuất hiện và bạn có thể quan sát quá trình mô phỏng. Log của bộ điều khiển sẽ được in ra trong terminal.

## 5. Phân tích Kết quả

Các file kết quả sẽ được lưu trong thư mục `output/`:

-   `tripinfo.xml`: Chứa thông tin chi tiết về mỗi chuyến đi (thời gian, quãng đường, thời gian chờ...).
-   `edgedata.xml`: Dữ liệu tổng hợp từ các cảm biến trên đường.
-   Các file `.csv` hoặc biểu đồ do các công cụ trong thư mục `tools/` tạo ra.

Ví dụ, để vẽ biểu đồ MFD (Macroscopic Fundamental Diagram) từ file `edgedata.xml`:

```bash
python tools/mfd_graph.py --file output/edgedata.xml --output output/mfd_graph.png
```
