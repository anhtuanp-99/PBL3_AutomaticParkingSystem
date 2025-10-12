#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

// ==========================
// Cấu hình chân
// ==========================
const int SERVO_PIN  = 18;   // GPIO18 - servo signal
const int BUTTON_PIN = 4;    // GPIO4  - nút nhấn
const int IR_PIN     = 5;    // GPIO5  - cảm biến hồng ngoại (OUT)

// ==========================
// Biến trạng thái
// ==========================
bool servoState = false;          // false = 0°, true = 90°
bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms chống dội

int lastIrState = HIGH; // lưu trạng thái trước của cảm biến

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);

  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(0);   // bắt đầu ở góc 0°
  Serial.println("Khoi dong xong: Barrier OFF (0 do), Khong co xe");
}

// ==========================
// Loop
// ==========================
void loop() {
  handleSystem();
}

// ==========================
// Hàm chính quản lý hệ thống
// ==========================
void handleSystem() {
  int irState = digitalRead(IR_PIN);

  // Chỉ xử lý khi có thay đổi trạng thái cảm biến
  if (irState != lastIrState) {
    if (irState == LOW) {
      Serial.println("Cam bien: CO XE");
      if (servoState) {
        Serial.println("Barrier -> OFF (dong 0 do)");
        moveServoSmooth(90, 0, 2000);
        servoState = false;
      }
    } else {
      Serial.println("Cam bien: KHONG CO XE");
      // Khi không có xe, nút nhấn có thể mở barrier
    }
    lastIrState = irState; // cập nhật trạng thái cũ
  }

  // Khi không có xe -> xử lý nút nhấn
  if (irState == HIGH) {
    handleButton();
  }
}

// ==========================
// Hàm xử lý nút nhấn (chống dội)
// ==========================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonStableState) {
      buttonStableState = reading;

      if (buttonStableState == LOW) {
        toggleServo();
      }
    }
  }

  lastButtonReading = reading;
}

// ==========================
// Hàm đổi trạng thái servo
// ==========================
void toggleServo() {
  int startAngle = servoState ? 90 : 0;
  servoState = !servoState;
  int targetAngle = servoState ? 90 : 0;

  Serial.print("Nut nhan: Barrier -> ");
  Serial.println(servoState ? "ON (90 do)" : "OFF (0 do)");

  moveServoSmooth(startAngle, targetAngle, 2000);
}

// ==========================
// Quay servo mượt
// ==========================
void moveServoSmooth(int start, int end, int duration) {
  int step = (end > start) ? 1 : -1;
  int steps = abs(end - start);
  int delayTime = duration / steps;

  for (int pos = start; pos != end; pos += step) {
    myServo.write(pos);
    delay(delayTime);
  }
  myServo.write(end);
  Serial.println("Hoan thanh chuyen dong servo");
}
