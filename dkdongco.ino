/*
 * motor_controller_mega.ino
 * Phiên bản tối ưu - Hỗ trợ rẽ chủ động
 * Viết bởi ChatGPT theo yêu cầu của Võ Trung Kiên
 * 
 * - Bình thường: L == R → cả hai tiến → đi thẳng
 * - Rẽ trái (R > L):   bánh trái LÙI, bánh phải TIẾN
 * - Rẽ phải (L > R):   bánh phải LÙI, bánh trái TIẾN
 * 
 * Giao thức Serial từ ROS2:
 *   "L:xxx, R:yyy\n"
 */

#include <Arduino.h>

// ======== Cấu hình chân BTS7960 ========
// ĐỘNG CƠ TRÁI
const int L_PWM_FWD = 5;
const int L_PWM_REV = 7;
const int L_EN_L    = 22;
const int L_EN_R    = 23;

// ĐỘNG CƠ PHẢI
const int R_PWM_FWD = 10;
const int R_PWM_REV = 11;
const int R_EN_L    = 24;
const int R_EN_R    = 25;


// ======== Giới hạn PWM [-255..255] ========
int constrainPWM(int value) {
  return constrain(value, -255, 255);
}


// ======== Bật EN ========
void enableMotors() {
  digitalWrite(L_EN_L, HIGH);
  digitalWrite(L_EN_R, HIGH);
  digitalWrite(R_EN_L, HIGH);
  digitalWrite(R_EN_R, HIGH);
}


// ======== Tắt động cơ ========
void stopMotors() {
  analogWrite(L_PWM_FWD, 0);
  analogWrite(L_PWM_REV, 0);
  analogWrite(R_PWM_FWD, 0);
  analogWrite(R_PWM_REV, 0);

  digitalWrite(L_EN_L, LOW);
  digitalWrite(L_EN_R, LOW);
  digitalWrite(R_EN_L, LOW);
  digitalWrite(R_EN_R, LOW);

  Serial.println("⚠️  Đã dừng động cơ.");
}


// ======== Hàm điều khiển động cơ + Auto Steering ========
void applyPWM(int pwmL, int pwmR) {
  pwmL = constrainPWM(pwmL);
  pwmR = constrainPWM(pwmR);

  enableMotors();

  // ================================
  // 🚗 AUTO–STEERING (TỰ ĐỘNG RẼ)
  // ================================
  if (pwmL == pwmR) {
    // → Đi thẳng
    Serial.println("➡️  Đi thẳng");
  }
  else if (pwmL > pwmR) {
    // → Rẽ phải → bánh phải LÙI
    Serial.println("↪️  Rẽ phải");
    pwmR = -abs(pwmR);
    pwmL = abs(pwmL);
  }
  else if (pwmR > pwmL) {
    // → Rẽ trái → bánh trái LÙI
    Serial.println("↩️  Rẽ trái");
    pwmL = -abs(pwmL);
    pwmR = abs(pwmR);
  }

  // ================================
  // Điều khiển động cơ thực tế
  // ================================
  // --- TRÁI ---
  if (pwmL >= 0) {
    analogWrite(L_PWM_FWD, pwmL);
    analogWrite(L_PWM_REV, 0);
  } else {
    analogWrite(L_PWM_FWD, 0);
    analogWrite(L_PWM_REV, -pwmL);
  }

  // --- PHẢI ---
  if (pwmR >= 0) {
    analogWrite(R_PWM_FWD, pwmR);
    analogWrite(R_PWM_REV, 0);
  } else {
    analogWrite(R_PWM_FWD, 0);
    analogWrite(R_PWM_REV, -pwmR);
  }

  Serial.print("   👉 PWM Trái = ");
  Serial.print(pwmL);
  Serial.print(" | PWM Phải = ");
  Serial.println(pwmR);
}


// ======== Tách chuỗi từ ROS2 ========
bool parseSerial(String input, int &pwmL, int &pwmR) {
  input.trim();

  int idxL = input.indexOf("L:");
  int idxR = input.indexOf("R:");
  int comma = input.indexOf(",");

  if (idxL < 0 || idxR < 0 || comma < 0) return false;

  String leftStr  = input.substring(idxL + 2, comma);
  String rightStr = input.substring(idxR + 2);

  pwmL = constrainPWM(leftStr.toInt());
  pwmR = constrainPWM(rightStr.toInt());

  return true;
}


// ======== Setup ========
void setup() {
  Serial.begin(115200);

  Serial.println("=== Bộ điều khiển động cơ Arduino đã sẵn sàng ===");

  // EN
  pinMode(L_EN_L, OUTPUT);
  pinMode(L_EN_R, OUTPUT);
  pinMode(R_EN_L, OUTPUT);
  pinMode(R_EN_R, OUTPUT);

  // PWM
  pinMode(L_PWM_FWD, OUTPUT);
  pinMode(L_PWM_REV, OUTPUT);
  pinMode(R_PWM_FWD, OUTPUT);
  pinMode(R_PWM_REV, OUTPUT);

  stopMotors();
}


// ======== Loop ========
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    int pwmL, pwmR;

    if (parseSerial(cmd, pwmL, pwmR)) {
      Serial.print("📥 Nhận lệnh: ");
      Serial.println(cmd);

      applyPWM(pwmL, pwmR);
    }
    else {
      Serial.print("❌ Lệnh không hợp lệ: ");
      Serial.println(cmd);
    }
  }
}