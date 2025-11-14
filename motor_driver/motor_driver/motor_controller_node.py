#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_controller_node.py
ROS2 Python node: subscribe /cmd_vel -> tính PWM -> gửi Arduino Mega qua Serial
Node name: motor_controller
Package: motor_driver
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None
    SerialException = Exception


def constrain(val, low, high):
    """Giới hạn giá trị val trong khoảng [low, high]"""
    if val is None:
        return 0
    if val < low:
        return low
    if val > high:
        return high
    return val


class SerialMotorInterface:
    """Quản lý kết nối Serial tới Arduino Mega, tự reconnect nếu ngắt"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, reconnect_interval=2.0, logger=None):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_interval = reconnect_interval
        self.logger = logger
        self._serial = None
        self._lock = threading.Lock()
        self._last_connect_attempt = 0.0

    def is_open(self):
        return self._serial is not None and getattr(self._serial, 'is_open', True)

    def _try_open(self):
        """Thử mở Serial, giảm tần suất theo reconnect_interval"""
        now = time.time()
        if self.is_open():
            return True
        if now - self._last_connect_attempt < self.reconnect_interval:
            return False
        self._last_connect_attempt = now

        if serial is None:
            if self.logger:
                self.logger.error("pyserial chưa cài. Chạy: pip install pyserial")
            return False

        try:
            if self.logger:
                self.logger.info(f"Thử mở cổng Serial {self.port} @ {self.baudrate}")
            ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(0.1)
            with self._lock:
                self._serial = ser
            if self.logger:
                self.logger.info(f"Đã mở cổng Serial {self.port}")
            return True
        except SerialException as e:
            if self.logger:
                self.logger.warn(f"Không mở được Serial {self.port}: {e}")
            with self._lock:
                self._serial = None
            return False
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Lỗi mở Serial: {e}")
            with self._lock:
                self._serial = None
            return False

    def send(self, s: str):
        """Gửi chuỗi s (thêm newline), trả về True nếu gửi thành công"""
        if not s.endswith('\n'):
            s += '\n'
        try:
            if not self.is_open():
                opened = self._try_open()
                if not opened:
                    return False
            with self._lock:
                if not self._serial:
                    return False
                self._serial.write(s.encode('utf-8'))
                try:
                    self._serial.flush()
                except Exception:
                    pass
            if self.logger:
                self.logger.debug(f"Gửi Serial: {s.strip()}")
            return True
        except SerialException as e:
            if self.logger:
                self.logger.warn(f"SerialException khi gửi: {e}")
            with self._lock:
                try:
                    if self._serial:
                        self._serial.close()
                except Exception:
                    pass
                self._serial = None
            return False
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Lỗi gửi Serial: {e}")
            with self._lock:
                try:
                    if self._serial:
                        self._serial.close()
                except Exception:
                    pass
                self._serial = None
            return False

    def close(self):
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        if self.logger:
            self.logger.info("Đóng cổng Serial.")


class MotorControllerNode(Node):
    """ROS2 node motor_controller: subscribe /cmd_vel -> gửi PWM tới Arduino"""

    def __init__(self):
        super().__init__('motor_controller')

        # Khai báo tham số
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.25)
        self.declare_parameter('pwm_scale', 100.0)
        self.declare_parameter('pwm_min', -255)
        self.declare_parameter('pwm_max', 255)
        self.declare_parameter('reconnect_interval', 2.0)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.pwm_scale = float(self.get_parameter('pwm_scale').value)
        self.pwm_min = int(self.get_parameter('pwm_min').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value)
        reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        self.get_logger().info(f"motor_controller start. Serial={serial_port}@{baudrate}, wheel_sep={self.wheel_separation}")

        self.serial_iface = SerialMotorInterface(port=serial_port, baudrate=baudrate,
                                                reconnect_interval=reconnect_interval, logger=self.get_logger())

        # Subscribe topic /cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        # Timer kiểm tra reconnect
        self.create_timer(max(0.1, reconnect_interval), self._periodic_check)

        self._last_sent = None

    def _periodic_check(self):
        if not self.serial_iface.is_open():
            self.serial_iface._try_open()

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Mô hình vi sai: v_left, v_right
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        pwm_left = int(constrain(v_left * self.pwm_scale, self.pwm_min, self.pwm_max))
        pwm_right = int(constrain(v_right * self.pwm_scale, self.pwm_min, self.pwm_max))

        cmd = f"L:{pwm_left},R:{pwm_right}"

        # Gửi nếu khác lệnh trước hoặc nếu Serial chưa mở
        if cmd != self._last_sent or not self.serial_iface.is_open():
            sent = self.serial_iface.send(cmd)
            if sent:
                self.get_logger().info(f"Đã gửi → {cmd}")
                self._last_sent = cmd
            else:
                self.get_logger().warn(f"Không gửi được (Serial chưa kết nối) → {cmd}")
        else:
            self.get_logger().debug(f"Bỏ qua lệnh trùng: {cmd}")

    def send_stop(self):
        """Gửi lệnh dừng động cơ"""
        stop_cmd = "L:0,R:0"
        try:
            sent = self.serial_iface.send(stop_cmd)
            if sent:
                self.get_logger().info("Đã gửi lệnh dừng L:0,R:0")
            else:
                self.get_logger().warn("Không gửi được lệnh dừng (Serial chưa kết nối)")
        except Exception as e:
            self.get_logger().warn(f"Lỗi khi gửi stop: {e}")

    def destroy_node(self):
        # Gửi stop trước khi tắt node
        self.send_stop()
        self.serial_iface.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
