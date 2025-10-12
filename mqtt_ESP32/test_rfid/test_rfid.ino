#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ---------- PIN (chỉnh theo kết nối của bạn) ----------
const uint8_t PIN_SCK  = 18; // SCK (VSPI)
const uint8_t PIN_MISO = 19; // MISO (VSPI)
const uint8_t PIN_MOSI = 23; // MOSI (VSPI)
const uint8_t PIN_SS   = 5;  // SDA / SS (RC522)
const uint8_t PIN_RST  = 2;  // RST (RC522)
const int SERVO_PIN = 22;    // Servo signal

// ---------- RFID & Servo ----------
MFRC522 rfid(PIN_SS, PIN_RST);
Servo myServo;

// ---------- Cấu hình ----------
String lastUid = "";
unsigned long lastTrigger = 0;
const unsigned long debounceMs = 5000; // chống lặp 5s
const unsigned long openDurationMs = 3000; // mở 3s
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// ---------- Hỗ trợ in dòng kẻ ----------
void printLine() {
  Serial.println("----------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  printLine();
  Serial.println("🔹 HỆ THỐNG RFID + SERVO (ESP32)");
  printLine();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  rfid.PCD_Init();
  Serial.println("✅ RC522 khởi tạo thành công!");

  myServo.attach(SERVO_PIN);
  myServo.write(CLOSED_ANGLE);
  Serial.print("✅ Servo gắn vào chân: ");
  Serial.println(SERVO_PIN);
  Serial.println("🔒 Servo ở trạng thái ĐÓNG (0°)");
  printLine();

  Serial.println("💡 Sẵn sàng. Hãy quét thẻ RFID...");
  printLine();
}

void loop() {
  // Chờ thẻ mới
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  // Đọc UID
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  printLine();
  Serial.println("✅ PHÁT HIỆN THẺ MỚI!");
  Serial.print("🆔 UID: ");
  Serial.println(uid);

  unsigned long now = millis();
  bool allowed = false;

  // Kiểm tra trùng UID trong thời gian debounce
  if (uid != lastUid || (now - lastTrigger >= debounceMs)) {
    allowed = true;
  }

  if (allowed) {
    Serial.println("⚙️  Mở barrier (servo → 90°)");
    lastUid = uid;
    lastTrigger = now;

    myServo.write(OPEN_ANGLE);
    Serial.println("⏱  Giữ 3 giây...");
    delay(openDurationMs);

    myServo.write(CLOSED_ANGLE);
    Serial.println("🔒 Barrier đóng lại (servo → 0°)");
  } else {
    Serial.println("⏸  Bỏ qua (đang trong thời gian chặn trùng UID)");
  }

  rfid.PICC_HaltA(); 
  rfid.PCD_StopCrypto1();

  printLine();
  Serial.println("💡 Chờ thẻ tiếp theo...");
  delay(300);
}
