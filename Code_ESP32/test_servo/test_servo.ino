#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 18;   // GPIO18 - servo signal
const int BUTTON_PIN = 4;   // GPIO4  - nút nhấn

bool servoState = false;    // false = 0°, true = 90°
bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms chống dội

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(0);
  Serial.println("Khoi dong: Servo o 0 do");
}

void loop() {
  handleButton();
}

// ==========================
// Hàm xử lý nút nhấn
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

  Serial.print("Dang quay servo tu ");
  Serial.print(startAngle);
  Serial.print(" den ");
  Serial.print(targetAngle);
  Serial.println(" trong 2s");

  moveServoSmooth(startAngle, targetAngle, 2000); // 2000ms = 2s
}

// ==========================
// Quay servo mượt
// ==========================
void moveServoSmooth(int start, int end, int duration) {
  int step = (end > start) ? 1 : -1;
  int steps = abs(end - start);
  int delayTime = duration / steps; // thời gian cho mỗi bước

  for (int pos = start; pos != end; pos += step) {
    myServo.write(pos);
    delay(delayTime);
  }
  myServo.write(end); // đảm bảo kết thúc đúng góc
  Serial.println("Hoan thanh");
}
