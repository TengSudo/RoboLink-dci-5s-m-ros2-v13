import time
from cri_lib import CRIController

# Khởi tạo CRIController
controller = CRIController()

# Kết nối đến iRC
if not controller.connect("192.168.3.11", 3920):
    print("Unable to connect")
    quit()

# Giành quyền điều khiển
controller.set_active_control(True)

print("Enable motors")
controller.enable()

print("Waiting for kinematics ready...")
controller.wait_for_kinematics_ready(10)

# Thiết lập tốc độ override
controller.set_override(100.0)

# ============================
# Nhập lệnh CMD từ terminal
# ============================
try:
    while True:
        cmd_input = input("Nhập lệnh CMD (hoặc 'exit' để thoát): ")

        if cmd_input.lower() == "exit":
            break

        full_cmd = f"CMD {cmd_input}"
        msg_id = controller._send_command(full_cmd, True)

        if msg_id is not None:
            print(f"Đã gửi lệnh: {full_cmd}")
            error = controller._wait_for_answer(str(msg_id), timeout=10)
            if error:
                print(f"Lỗi từ robot: {error}")
            else:
                print("Lệnh thực hiện thành công.")
        else:
            print("Không thể gửi lệnh.")

except KeyboardInterrupt:
    print("\nDừng chương trình.")

# Tắt động cơ và ngắt kết nối
print("Tắt động cơ và ngắt kết nối...")
controller.disable()
controller.close()
