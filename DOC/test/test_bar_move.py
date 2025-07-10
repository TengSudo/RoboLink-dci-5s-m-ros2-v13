import tkinter as tk
from cri_lib import CRIController

# Tạo cửa sổ giao diện
root = tk.Tk()
root.title("Robot Control Interface")

# Khởi tạo đối tượng điều khiển robot
controller = CRIController()

# Kết nối tới robot (đảm bảo rằng robot đang chạy và có thể kết nối)
if not controller.connect("192.168.3.11", 3920):
    print("Unable to connect")
    quit()

#acquire active control.
controller.set_active_control(True)

print("enable")
#enable motors
controller.enable()

print("waiting")
#wait until kinematics are ready to move
controller.wait_for_kinematics_ready(10)

controller.set_override(100.0)

# Khởi tạo các giá trị mặc định cho các trục
x_value = 250.0
y_value = 0.0
z_value = 250.0

# Hàm điều khiển robot khi thay đổi giá trị từ thanh trượt
def update_robot_position(*args):
    # Lấy giá trị từ các thanh trượt
    x_pos = x_slider.get()
    y_pos = y_slider.get()
    z_pos = z_slider.get()
    
    # Di chuyển robot đến vị trí mới
    controller.move_cartesian(x_pos, y_pos, z_pos, 180.0, 0.0, 180.0, 0.0, 0.0, 0.0, 30.0, wait_move_finished=False)
    print(f"Moving to: X={x_pos} Y={y_pos} Z={z_pos}")

# Giao diện với các thanh trượt cho các trục X, Y, Z
x_slider = tk.Scale(root, from_=150, to_=350, orient="horizontal", label="X Position (150-350)", length=300)
x_slider.set(x_value)  # Set giá trị mặc định cho X
x_slider.pack(pady=10)

y_slider = tk.Scale(root, from_=-100, to_=100, orient="horizontal", label="Y Position (-100 to 100)", length=300)
y_slider.set(y_value)  # Set giá trị mặc định cho Y
y_slider.pack(pady=10)

z_slider = tk.Scale(root, from_=150, to_=300, orient="horizontal", label="Z Position (150 to 300)", length=300)
z_slider.set(z_value)  # Set giá trị mặc định cho Z
z_slider.pack(pady=10)

# Liên kết các thanh trượt với hàm update_robot_position
x_slider.bind("<Motion>", update_robot_position)
y_slider.bind("<Motion>", update_robot_position)
z_slider.bind("<Motion>", update_robot_position)

# Chạy giao diện Tkinter
root.mainloop()
