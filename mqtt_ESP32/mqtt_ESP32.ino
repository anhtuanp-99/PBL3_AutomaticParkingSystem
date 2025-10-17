#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ==========================
// WiFi & MQTT
// ==========================
const char* ssid = "Kaka";
const char* password = "12345678";
const char* mqtt_server = "b5730ad18edf4f60acae31bb8160b04d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "my_cloud";
const char* mqtt_password = "Anhtuan123";

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ==========================
// PIN mapping (xác nhận bởi bạn)
// ==========================
const int SERVO_PIN = 21;
const int BUTTON_PIN = 4;
const int IR_PIN_MAIN = 34;               // cảm biến sau barrier (CO XE = LOW)
const int IR_PARK[4] = {13, 33, 14, 32};  // P1..P4

// RFID (RC522)
const uint8_t PIN_SCK  = 18;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_SS   = 5;
const uint8_t PIN_RST  = 2;

MFRC522 rfid(PIN_SS, PIN_RST);
Servo myServo;

// ==========================
// Thời gian / trạng thái
// ==========================
bool servoState = false; // false = closed(0°), true = open(90°)

// Button debounce
bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Barrier IR (lọc 300 ms)
int lastIrReadingMain = HIGH;      // giá trị đọc gần nhất
int stableIrMain = HIGH;           // trạng thái đã xác nhận
unsigned long lastChangeTimeMain = 0;
const unsigned long barrierConfirmDelay = 200; // 300 ms chống chớp

// Flags controlling RFID-open behavior
bool openedByRFID = false;         // true nếu barrier vừa mở do RFID và đang chờ xe qua
bool carDetectedAfterOpen = false; // true nếu IR_MAIN đọc LOW sau khi mở (xe đã vào vùng)

// Parking slots (lọc 3000 ms)
struct ParkingSlot {
  int pin;
  int lastState;
  int stableState;
  unsigned long lastChangeTime;
};
ParkingSlot slots[4];
const unsigned long confirmDelay = 3000; // 3s

// RFID debounce
String lastUid = "";
unsigned long lastTrigger = 0;
const unsigned long debounceMs = 5000; // 5s để tránh quét lặp

// Servo angles
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// ==========================
// Helper: smooth move servo
// ==========================
void moveServoSmooth(int start, int end, int duration) {
  if (start == end) return;
  int step = (end > start) ? 1 : -1;
  int steps = abs(end - start);
  int delayTime = (steps == 0) ? 0 : duration / steps;
  for (int pos = start; pos != end; pos += step) {
    myServo.write(pos);
    if (delayTime > 0) delay(delayTime);
  }
  myServo.write(end);
}

// ==========================
// Publish helper
// ==========================
void mqttPublish(const char* topic, const char* msg) {
  if (client.connected()) {
    client.publish(topic, msg);
  }
  Serial.printf("📤 MQTT [%s] => %s\n", topic, msg);
}

// ==========================
// Toggle servo (manual/MQTT). Does NOT trigger RFID-close logic.
// ==========================
void setServoState(bool open, const char* source) {
  if (servoState == open) {
    Serial.printf("ℹ️ Servo already %s (requested by %s)\n", open ? "OPEN" : "CLOSED", source);
    return;
  }
  int startAngle = servoState ? OPEN_ANGLE : CLOSED_ANGLE;
  int targetAngle = open ? OPEN_ANGLE : CLOSED_ANGLE;
  moveServoSmooth(startAngle, targetAngle, 800);
  servoState = open;
  // If manually opened/closed (button or MQTT), we treat openedByRFID = false
  openedByRFID = false;
  carDetectedAfterOpen = false;
  String msg = String("Barrier ") + (open ? "OPEN" : "CLOSE") + " (" + source + ")";
  mqttPublish("status", msg.c_str());
  Serial.println(msg);
}

// ==========================
// Button handling (manual open/close)
// ==========================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonStableState) {
      buttonStableState = reading;
      if (buttonStableState == LOW) {
        // Toggle manually
        setServoState(!servoState, "Button");
      }
    }
  }
  lastButtonReading = reading;
}

// ==========================
// Barrier IR handling with 300ms confirm
// Logic special: barrier closes automatically only when it was opened by RFID
// and IR sees LOW (car present) then returns to HIGH (car passed) stabilized.
// ==========================
void handleIR_Main() {
  int reading = digitalRead(IR_PIN_MAIN);

  // detect raw change and record time
  if (reading != lastIrReadingMain) {
    lastChangeTimeMain = millis();
    lastIrReadingMain = reading;
  }

  // confirm if stable for barrierConfirmDelay
  if ((millis() - lastChangeTimeMain) >= barrierConfirmDelay) {
    if (reading != stableIrMain) {
      // stable state changed
      stableIrMain = reading;

      if (stableIrMain == LOW) {
        // Car present at sensor
        Serial.println("🚗 IR_MAIN: CO XE (stable)");
        mqttPublish("parking/barrier", "CO XE");
        // mark that car passed into sensor after an RFID open
        if (openedByRFID) {
          carDetectedAfterOpen = true;
          Serial.println("🔔 Car detected after RFID-open -> waiting for car to leave to close barrier");
        }
      } else {
        // stable HIGH = no car at sensor
        Serial.println("🅿️ IR_MAIN: TRONG (stable)");
        mqttPublish("parking/barrier", "TRONG");
        // If previously opened by RFID and we already detected car present, now it's left -> close
        if (openedByRFID && carDetectedAfterOpen) {
          Serial.println("✅ Car passed through -> closing barrier (triggered by IR_MAIN)");
          setServoState(false, "IR_Passed");
          // publish specific close reason
          mqttPublish("status", "Barrier CLOSE (IR Passed)");
          // reset flags
          openedByRFID = false;
          carDetectedAfterOpen = false;
        }
      }
    }
  }

  // allow manual button only when no car at sensor (stable)
  if (stableIrMain == HIGH) handleButton();
}

// ==========================
// Parking slot handler (3s confirm)
// ==========================
void handleParkingSlot(int i) {
  int reading = digitalRead(slots[i].pin);

  if (reading != slots[i].lastState) {
    slots[i].lastChangeTime = millis();
    slots[i].lastState = reading;
  }

  if ((millis() - slots[i].lastChangeTime) >= confirmDelay) {
    if (slots[i].stableState != reading) {
      slots[i].stableState = reading;

      String msg = "P" + String(i + 1) + " ";
      msg += (reading == LOW) ? "CO XE" : "TRONG";

      String topic = "parking/slot" + String(i + 1);
      mqttPublish(topic.c_str(), msg.c_str());
      Serial.printf("🏁 %s -> %s\n", topic.c_str(), msg.c_str());
    }
  }
}

// ==========================
// RFID handling
// - On detection: open barrier (no auto-close here).
// - Set openedByRFID = true and carDetectedAfterOpen = false.
// - Debounce to avoid repeated triggers.
// ==========================
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  Serial.println("----------------------------------");
  Serial.println("✅ RFID detected");
  Serial.print("🆔 UID: ");
  Serial.println(uid);

  unsigned long now = millis();
  if (uid != lastUid || (now - lastTrigger >= debounceMs)) {
    lastUid = uid;
    lastTrigger = now;

    // Open barrier if currently closed
    if (!servoState) {
      Serial.println("⚙️ Opening barrier (RFID request)...");
      setServoState(true, "RFID");
      // Set the RFID-open flow: wait for car detection then close
      openedByRFID = true;
      carDetectedAfterOpen = false;
      mqttPublish("status", "Barrier OPEN (RFID)");
      // Do NOT auto-close here; closing will happen after IR detection sequence
    } else {
      Serial.println("ℹ️ Barrier already open; RFID scan accepted but no action taken.");
    }
  } else {
    Serial.println("⏸ RFID ignored (debounce)");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ==========================
// MQTT callback
// ==========================
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[100];
  unsigned int copyLen = min(length, (unsigned int)sizeof(msg) - 1);
  for (unsigned int i = 0; i < copyLen; i++) msg[i] = (char)payload[i];
  msg[copyLen] = '\0';

  Serial.printf("📩 MQTT [%s]: %s\n", topic, msg);

  if (strcmp(topic, "barrier") == 0) {
    if (strcmp(msg, "OPEN") == 0) {
      // only open if sensor is clear (no car blocking after barrier)
      if (stableIrMain == HIGH) {
        setServoState(true, "MQTT");
      } else {
        Serial.println("⚠️ MQTT OPEN ignored: IR_MAIN busy");
      }
    } else if (strcmp(msg, "CLOSE") == 0) {
      // manual close request allowed only if no car currently at sensor
      if (stableIrMain == HIGH) {
        setServoState(false, "MQTT");
      } else {
        Serial.println("⚠️ MQTT CLOSE ignored: IR_MAIN busy");
      }
    }
  }
}

// ==========================
// MQTT reconnect
// ==========================
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔗 Connecting to MQTT...");
    String clientID = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("✅ MQTT connected");
      client.subscribe("barrier");
    } else {
      Serial.printf("❌ MQTT failed (%d). Retry in 5s\n", client.state());
      delay(5000);
    }
  }
}

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("----------------------------------");
  Serial.println("🔹 SYSTEM START: RFID + Barrier + IR + MQTT");
  Serial.println("----------------------------------");

  // SPI for RC522: pass SCK, MISO, MOSI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  rfid.PCD_Init();
  Serial.println("✅ RC522 initialized");

  // pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_PIN_MAIN, INPUT);
  for (int i = 0; i < 4; i++) {
    slots[i].pin = IR_PARK[i];
    pinMode(slots[i].pin, INPUT);
    slots[i].lastState = HIGH;
    slots[i].stableState = HIGH;
    slots[i].lastChangeTime = 0;
  }

  // servo
  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(CLOSED_ANGLE);
  servoState = false;
  Serial.printf("✅ Servo attached to pin %d (closed)\n", SERVO_PIN);

  // WiFi + MQTT
  WiFi.begin(ssid, password);
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.print("📶 Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());
}

// ==========================
// Main loop
// ==========================
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // IR main handles button internally
  handleIR_Main();

  // parking slots
  for (int i = 0; i < 4; i++) handleParkingSlot(i);

  // RFID
  handleRFID();

  // small yield
  delay(5);
}
