
import matplotlib.pyplot as plt
from typing import List

class RealTimePlotter:
    """
    Một lớp để vẽ đồ thị dữ liệu theo thời gian thực bằng Matplotlib mà không chặn luồng chính.
    """
    def __init__(self, set_point: float, title: str = "Accumulation Plot", y_label: str = "Tích lũy (n(k))", x_label: str = "Thời gian (s)"):
        plt.ion()  # Bật chế độ tương tác
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.set_point = set_point
        
        # Lưu trữ dữ liệu
        self.time_data: List[float] = []
        self.nk_data: List[float] = []
        
        # Các thành phần của đồ thị
        self.line, = self.ax.plot(self.time_data, self.nk_data, 'b-', label='Tích lũy n(k)') 
        self.set_point_line = self.ax.axhline(y=self.set_point, color='r', linestyle='--', label=f'Điểm đặt n_hat={self.set_point:.0f}')
        
        # Nhãn và tiêu đề
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)
        self.ax.set_title(title)
        self.ax.legend()
        self.ax.grid(True)
        self.fig.tight_layout()

    def update_plot(self, time_step: float, nk_value: float):
        """Cập nhật đồ thị với một điểm dữ liệu mới."""
        # Thêm dữ liệu mới
        self.time_data.append(time_step)
        self.nk_data.append(nk_value)
        
        # Cập nhật dữ liệu cho đường vẽ
        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.nk_data)
        
        # Tự động điều chỉnh lại giới hạn của các trục
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Vẽ lại canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Tạm dừng một chút để giao diện đồ họa có thời gian cập nhật
        plt.pause(0.01)

    def close(self):
        """Giữ đồ thị hiển thị sau khi mô phỏng kết thúc."""
        plt.ioff() # Tắt chế độ tương tác
        plt.show() # Hiển thị đồ thị cuối cùng (hành động này sẽ chặn cho đến khi người dùng đóng cửa sổ)
