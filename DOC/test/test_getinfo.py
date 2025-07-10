import logging
import time
from cri_lib import CRIController


def main():
    controller = CRIController()

    host = "192.168.3.11"  # Thay địa chỉ IP của robot của bạn
    port = 3920  # Thay port nếu cần
    if controller.connect(host, port):
        logging.info("Robot đã kết nối thành công!")

        try:
            while True:
                position = controller.robot_state.joints_current
                logging.info(f"{position.A1} {position.A2} {position.A3} {position.A4} {position.A5} {position.A6}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            logging.info("Vòng lặp bị ngắt bởi người dùng.")
        controller.close()
    else:
        logging.error("Không thể kết nối với robot.")
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
