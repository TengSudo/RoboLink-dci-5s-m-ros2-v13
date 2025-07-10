#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer
from cri_lib import CRIController, CRIConnectionError
from me5110q_gui.untitled import Ui_ME5110Q  

class ME5110QGUINode(Node):
    def __init__(self, app):
        super().__init__('me5110q_gui_node')
        self.app = app
        
        # Tạo cửa sổ chính
        self.main_window = QtWidgets.QMainWindow()
        self.ui = Ui_ME5110Q()
        self.ui.setupUi(self.main_window)
        
        # Khởi tạo CRIController
        self.robot_controller = None
        
        # Kết nối các sự kiện
        self.connect_signals()
        
        # Hiển thị cửa sổ
        self.main_window.show()
        self.get_logger().info("ME5110Q GUI Node đã khởi chạy")
        self.append_log("ME5110Q GUI Node đã khởi chạy")

        # Thiết lập phạm vi cho slider_override
        self.ui.slider_override.setMinimum(0)
        self.ui.slider_override.setMaximum(100)
        self.ui.slider_override.setValue(0)  # Giá trị mặc định

        # Timer để xử lý sự kiện ROS2 trong Qt event loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_ros_timer)
        self.timer.start(100)  # 100ms

        # Timer để cập nhật trạng thái
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_labels)
        self.status_timer.setInterval(100)  # Cập nhật mỗi 100ms

    def append_log(self, message):
        """Thêm thông điệp log vào textEdit_log với thời gian hiện tại"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        self.ui.textEdit_log.append(log_message)
        # Tự động cuộn xuống dòng mới nhất
        self.ui.textEdit_log.verticalScrollBar().setValue(
            self.ui.textEdit_log.verticalScrollBar().maximum()
        )

    def connect_signals(self):
        """Kết nối các sự kiện GUI"""
        # Camera
        self.ui.pushButton_camera_connect.clicked.connect(self.camera_connect)
        self.ui.pushButton_camera_disconnet.clicked.connect(self.camera_disconnect)
        
        # Robot
        self.ui.pushButton_robot_connect.clicked.connect(self.robot_connect)
        self.ui.pushButton_robot_disconnect.clicked.connect(self.robot_disconnect)
        self.ui.pushButton_reset.clicked.connect(self.robot_reset)
        self.ui.pushButton_enable.clicked.connect(self.robot_enable)
        self.ui.pushButton_ref.clicked.connect(self.robot_reference)
        self.ui.pushButton_gripper_close.clicked.connect(self.close_gripper)
        self.ui.pushButton_gripper_open.clicked.connect(self.open_gripper)
        self.ui.slider_override.valueChanged.connect(self.robot_override_changed)

    def on_ros_timer(self):
        """Xử lý các sự kiện ROS2 trong Qt event loop"""
        rclpy.spin_once(self, timeout_sec=0)

    def update_status_labels(self):
        """Cập nhật trạng thái lỗi, E-Stop, kinematics, vị trí cartesian, vị trí khớp, và trạng thái gripper"""
        if self.robot_controller is not None and self.robot_controller.connected:
            # Lấy trạng thái từ robot_state
            error_state = self.robot_controller.robot_state.combined_axes_error
            estop_state = str(self.robot_controller.robot_state.emergency_stop_ok)
            kin_state = str(self.robot_controller.robot_state.kinematics_state)
            # Vị trí cartesian (X, Y, Z, A, B, C)
            pos_cart = self.robot_controller.robot_state.position_robot
            pos_cart_str = (f"X_{pos_cart.X:.2f} Y_{pos_cart.Y:.2f} Z_{pos_cart.Z:.2f} "
                           f"A_{pos_cart.A:.2f} B_{pos_cart.B:.2f} C_{pos_cart.C:.2f}")
            # Vị trí khớp (A1, A2, A3, A4, A5, A6)
            pos_joints = self.robot_controller.robot_state.joints_current
            pos_joints_str = (f"A1_{pos_joints.A1:.2f} A2_{pos_joints.A2:.2f} A3_{pos_joints.A3:.2f} "
                             f"A4_{pos_joints.A4:.2f} A5_{pos_joints.A5:.2f} A6_{pos_joints.A6:.2f}")
            # Trạng thái DOUT 20 (gripper)
            gripper_state = str(self.robot_controller.robot_state.dout[20])
            
            # Cập nhật các label trên GUI
            self.ui.label_error.setText(f"Error: {error_state}")
            self.ui.label_estop.setText(f"E-Stop: {estop_state}")
            self.ui.label_kinstate.setText(f"{kin_state}")
            self.ui.label_pos_cart.setText(f"PosCart: {pos_cart_str}")
            self.ui.label_pos_joints.setText(f"PosJoints: {pos_joints_str}")
            self.ui.label_gripper.setText(f"Gripper: {gripper_state}")
        else:
            # Nếu không kết nối, đặt lại các label
            self.ui.label_error.setText("Error: Không kết nối")
            self.ui.label_estop.setText("E-Stop: Không kết nối")
            self.ui.label_kinstate.setText("KinState: Không kết nối")
            self.ui.label_pos_cart.setText("PosCart: Không kết nối")
            self.ui.label_pos_joints.setText("PosJoints: Không kết nối")
            self.ui.label_gripper.setText("Gripper: Không kết nối")

    def camera_connect(self):
        self.get_logger().info("Kết nối camera...")
        self.ui.label_status_camera.setText("Status: Đã kết nối")
        self.append_log("Kết nối camera...")

    def camera_disconnect(self):
        self.get_logger().info("Ngắt kết nối camera...")
        self.ui.label_status_camera.setText("Status: Ngắt kết nối")
        self.append_log("Ngắt kết nối camera...")

    def robot_connect(self):
        """Kết nối với robot sử dụng CRIController"""
        ip = self.ui.lineEdit_ip.text()
        try:
            port = int(self.ui.lineEdit_port.text())
        except ValueError:
            self.get_logger().error("Cổng không hợp lệ, vui lòng nhập số nguyên.")
            self.ui.label_robot_status.setText("Status: Lỗi - Cổng không hợp lệ")
            self.append_log("Lỗi: Cổng không hợp lệ, vui lòng nhập số nguyên.")
            return

        # Khởi tạo CRIController nếu chưa được khởi tạo
        if self.robot_controller is None:
            self.robot_controller = CRIController()

        try:
            # Thử kết nối với robot
            if self.robot_controller.connect(ip, port):
                self.get_logger().info(f"Đã kết nối với robot tại {ip}:{port}")
                self.ui.label_robot_status.setText("Status: Đã kết nối")
                self.append_log(f"Đã kết nối với robot tại {ip}:{port}")
                # Điều chỉnh slider_override về 30% sau khi kết nối thành công
                self.ui.slider_override.setValue(30)
                # Vô hiệu hóa nút kết nối
                self.ui.pushButton_robot_connect.setEnabled(False)
                # Bắt đầu timer cập nhật trạng thái
                self.status_timer.start()
            else:
                self.get_logger().error(f"Kết nối thất bại tại {ip}:{port}")
                self.ui.label_robot_status.setText("Status: Kết nối thất bại")
                self.append_log(f"Kết nối thất bại tại {ip}:{port}")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi kết nối: {str(e)}")
            self.ui.label_robot_status.setText("Status: Lỗi kết nối")
            self.append_log(f"Lỗi kết nối: {str(e)}")

    def robot_disconnect(self):
        """Ngắt kết nối với robot"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().info("Không có kết nối robot để ngắt")
            self.ui.label_robot_status.setText("Status: Ngắt kết nối")
            self.append_log("Không có kết nối robot để ngắt")
            # Kích hoạt lại nút kết nối
            self.ui.pushButton_robot_connect.setEnabled(True)
            # Dừng timer cập nhật trạng thái
            self.status_timer.stop()
            # Đặt lại các label trạng thái
            self.ui.label_error.setText("Error: Không kết nối")
            self.ui.label_estop.setText("ESTOP: Không kết nối")
            return

        try:
            self.robot_controller.close()
            self.get_logger().info("Đã ngắt kết nối robot")
            self.ui.label_robot_status.setText("Status: Ngắt kết nối")
            self.append_log("Đã ngắt kết nối robot")
            # Kích hoạt lại nút kết nối
            self.ui.pushButton_robot_connect.setEnabled(True)
            # Dừng timer cập nhật trạng thái
            self.status_timer.stop()
            # Đặt lại các label trạng thái
            self.ui.label_error.setText("Error: Không kết nối")
            self.ui.label_estop.setText("ESTOP: Không kết nối")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi ngắt kết nối: {str(e)}")
            self.ui.label_robot_status.setText("Status: Lỗi ngắt kết nối")
            self.append_log(f"Lỗi khi ngắt kết nối: {str(e)}")
        finally:
            self.robot_controller = None

    def robot_reset(self):
        """Gọi phương thức reset của robot"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().error("Không thể reset: Robot chưa được kết nối")
            self.append_log("Lỗi: Không thể reset, robot chưa được kết nối")
            return

        try:
            if self.robot_controller.reset():
                self.get_logger().info("Đã reset robot thành công")
                self.append_log("Đã reset robot thành công")
            else:
                self.get_logger().error("Reset robot thất bại")
                self.append_log("Reset robot thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi reset robot: {str(e)}")
            self.append_log(f"Lỗi khi reset robot: {str(e)}")

    def robot_enable(self):
        """Gọi phương thức enable của robot"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().error("Không thể enable: Robot chưa được kết nối")
            self.append_log("Lỗi: Không thể enable, robot chưa được kết nối")
            return

        try:
            if self.robot_controller.enable():
                self.get_logger().info("Đã enable robot thành công")
                self.append_log("Đã enable robot thành công")
            else:
                self.get_logger().error("Enable robot thất bại")
                self.append_log("Enable robot thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi enable robot: {str(e)}")
            self.append_log(f"Lỗi khi enable robot: {str(e)}")

    def robot_reference(self):
        """Gọi phương thức tham chiếu robot dựa trên lựa chọn trong combo_ref"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().error("Không thể tham chiếu: Robot chưa được kết nối")
            self.append_log("Lỗi: Không thể tham chiếu, robot chưa được kết nối")
            return

        selected_option = self.ui.combo_ref.currentText()
        try:
            if selected_option == "All joints":
                if self.robot_controller.reference_all_joints():
                    self.get_logger().info("Đã tham chiếu tất cả các khớp thành công")
                    self.append_log("Đã tham chiếu tất cả các khớp thành công")
                else:
                    self.get_logger().error("Tham chiếu tất cả các khớp thất bại")
                    self.append_log("Tham chiếu tất cả các khớp thất bại")
            else:
                # Ánh xạ từ "Joint 1" -> "A1", "Joint 2" -> "A2", v.v.
                joint_map = {
                    "Joint 1": "A1",
                    "Joint 2": "A2",
                    "Joint 3": "A3",
                    "Joint 4": "A4",
                    "Joint 5": "A5"
                }
                joint = joint_map.get(selected_option)
                if joint and self.robot_controller.reference_single_joint(joint):
                    self.get_logger().info(f"Đã tham chiếu khớp {selected_option} thành công")
                    self.append_log(f"Đã tham chiếu khớp {selected_option} thành công")
                else:
                    self.get_logger().error(f"Tham chiếu khớp {selected_option} thất bại")
                    self.append_log(f"Tham chiếu khớp {selected_option} thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi tham chiếu robot: {str(e)}")
            self.append_log(f"Lỗi khi tham chiếu robot: {str(e)}")

    def robot_override_changed(self):
        """Gọi phương thức set_override khi giá trị slider_override thay đổi"""
        if self.robot_controller is None or not self.robot_controller.connected:
            return

        override_value = self.ui.slider_override.value()
        try:
            if self.robot_controller.set_override(override_value):
                self.get_logger().info(f"Đã đặt override thành {override_value}%")
                self.ui.label_override.setText(f"Override {override_value}%")
            else:
                self.get_logger().error(f"Đặt override {override_value}% thất bại")
                self.ui.label_override.setText("Override thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi đặt override: {str(e)}")

    def close_gripper(self):
        """Đóng kẹp robot"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().error("Không thể đóng kẹp: Robot chưa được kết nối")
            self.append_log("Lỗi: Không thể đóng kẹp, robot chưa được kết nối")
            return

        try:
            if self.robot_controller.set_dout(22, True):
                self.get_logger().info("Đã đóng kẹp thành công")
                self.append_log("Đã đóng kẹp thành công")
            else:
                self.get_logger().error("Đóng kẹp thất bại")
                self.append_log("Đóng kẹp thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi đóng kẹp: {str(e)}")
            self.append_log(f"Lỗi khi đóng kẹp: {str(e)}")

    def open_gripper(self):
        """Mở kẹp robot"""
        if self.robot_controller is None or not self.robot_controller.connected:
            self.get_logger().error("Không thể mở kẹp: Robot chưa được kết nối")
            self.append_log("Lỗi: Không thể mở kẹp, robot chưa được kết nối")
            return

        try:
            if self.robot_controller.set_dout(22, False):
                self.get_logger().info("Đã mở kẹp thành công")
                self.append_log("Đã mở kẹp thành công")
            else:
                self.get_logger().error("Mở kẹp thất bại")
                self.append_log("Mở kẹp thất bại")
        except CRIConnectionError as e:
            self.get_logger().error(f"Lỗi khi mở kẹp: {str(e)}")
            self.append_log(f"Lỗi khi mở kẹp: {str(e)}")

def main(args=None):
    # Khởi tạo Qt Application trước
    app = QtWidgets.QApplication(sys.argv)
    
    # Khởi tạo ROS2
    rclpy.init(args=args)
    
    try:
        # Tạo node
        node = ME5110QGUINode(app)
        
        # Chạy ứng dụng Qt
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()