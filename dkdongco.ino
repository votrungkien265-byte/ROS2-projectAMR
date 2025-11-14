/*
 * motor_controller_mega.ino
 * Sketch Arduino Mega để điều khiển 2 động cơ DC qua BTS7960
 * Nhận lệnh từ ROS2 Python Node qua Serial
 * Tương thích node: motor_controller_node.py
 * Tất cả comment và log hiển thị bằng tiếng Việt
 */

#include <Arduino.h>

// ======== Cấu hình chân kết nối BTS7960 ========
const int L_PWM_FWD = 5;  // PWM tiến động cơ trái
const int L_PWM_REV = 6;  // PWM lùi động cơ trái
const int R_PWM_FWD = 10; // PWM tiến động cơ phải
const int R_PWM_REV = 11; // PWM lùi động cơ phải

// ======== Biến lưu trữ PWM hiện tại ========
int pwmLeft = 0;
int pwmRight = 0;

// ======== Giới hạn giá trị PWM [-255, 255] ========
int constrainPWM(int pwm) {
  if (pwm > 255) return 255;
  if (pwm < -255) return -255;
  return pwm;
}

// ======== Hàm áp dụng PWM cho động cơ ========
void applyPWM(int pwmL, int pwmR) {
  pwmL = constrainPWM(pwmL);
  pwmR = constrainPWM(pwmR);

  // Động cơ trái
  if (pwmL > 0) { // Chạy thuận
    analogWrite(L_PWM_FWD, pwmL);
    analogWrite(L_PWM_REV, 0);
  } else if (pwmL < 0) { // Chạy ngược
    analogWrite(L_PWM_FWD, 0);
    analogWrite(L_PWM_REV, -pwmL);
  } else { // Dừng
    analogWrite(L_PWM_FWD, 0);
    analogWrite(L_PWM_REV, 0);
  }

  // Động cơ phải
  if (pwmR > 0) { // Chạy thuận
    analogWrite(R_PWM_FWD, pwmR);
    analogWrite(R_PWM_REV, 0);
  } else if (pwmR < 0) { // Chạy ngược
    analogWrite(R_PWM_FWD, 0);
    analogWrite(R_PWM_REV, -pwmR);
  } else { // Dừng
    analogWrite(R_PWM_FWD, 0);
    analogWrite(R_PWM_REV, 0);
  }
}

// ======== Hàm dừng tất cả động cơ ========
void stopMotors() {
  applyPWM(0, 0);
  Serial.println("Động cơ đã dừng");
}

// ======== Hàm đọc chuỗi Serial và tách PWM ========
bool parseSerial(String input, int &pwmL, int &pwmR) {
  input.trim(); // loại bỏ khoảng trắng và \r\n

  int lIndex = input.indexOf("L:");
  int rIndex = input.indexOf("R:");

  if (lIndex == -1 || rIndex == -1) return false;

  String lStr = input.substring(lIndex + 2, input.indexOf(',', lIndex));
  String rStr = input.substring(rIndex + 2);

  pwmL = lStr.toInt();
  pwmR = rStr.toInt();

  pwmL = constrainPWM(pwmL);
  pwmR = constrainPWM(pwmR);

  return true;
}

// ======== Setup ========
void setup() {
  // Cấu hình chân PWM là OUTPUT
  pinMode(L_PWM_FWD, OUTPUT);
  pinMode(L_PWM_REV, OUTPUT);
  pinMode(R_PWM_FWD, OUTPUT);
  pinMode(R_PWM_REV, OUTPUT);

  // Khởi động Serial
  Serial.begin(115200);
  Serial.println("=== Arduino Motor Controller Sẵn sàng ===");
}

// ======== Loop chính ========
void loop() {
  // Kiểm tra Serial có dữ liệu mới không
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n'); // đọc từng dòng

    int lPWM, rPWM;

    if (parseSerial(line, lPWM, rPWM)) {
      pwmLeft = lPWM;
      pwmRight = rPWM;
      applyPWM(pwmLeft, pwmRight);

      // Echo log để ROS2 node debug
      Serial.print("Đã nhận lệnh: L:");
      Serial.print(pwmLeft);
      Serial.print(", R:");
      Serial.println(pwmRight);

    } else {
      // Nếu chuỗi sai → dừng động cơ
      stopMotors();
      Serial.println("Lỗi: Lệnh Serial không hợp lệ!");
    }
  }
}
