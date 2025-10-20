#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ====== WiFi & MQTT ======
const char* ssid = "Kaka";
const char* password = "12345678";
const char* mqtt_server = "b5730ad18edf4f60acae31bb8160b04d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "my_cloud";
const char* mqtt_password = "Anhtuan123";
WiFiClientSecure espClient;
PubSubClient client(espClient);

// ====== PIN mapping ======
const int RFID_SCK = 18, RFID_MISO = 19, RFID_MOSI = 23, RFID_SS = 5, RFID_RST = 2;

// Barrier VÀO
const int IR_TRUOC_VAO = 15;
const int IR_SAU_VAO   = 34;
const int SERVO_VAO_PIN = 21;

// Barrier RA
const int IR_TRUOC_RA = 26;
const int IR_SAU_RA   = 27;
const int SERVO_RA_PIN = 25;

// Parking slots
const int IR_PARK[4] = {13, 33, 14, 32};

// Servo góc
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// ====== RFID ======
MFRC522 rfid(RFID_SS, RFID_RST);

// ====== Servo ======
Servo servoVao, servoRa;
bool servoVaoState = false;
bool servoRaState = false;

// ====== Thời gian / debounce ======
const unsigned long CONFIRM_MS = 200;
const unsigned long SLOT_CONFIRM_MS = 3000;
const unsigned long RFID_DEBOUNCE_MS = 5000;
int BARRIER_SPEED_MS = 800;

// ====== Cấu trúc theo dõi IR ======
struct IRTracker {
  int pin;
  int lastRead;
  int stable;
  unsigned long lastChange;
  bool sawLowAfterOpen;
} beforeVao, afterVao, beforeRa, afterRa, slots[4];

bool openedByRFID_Vao = false;
bool openedByRFID_Ra  = false;

// ====== MQTT helper ======
void mqttPublish(const char* topic, const char* msg) {
  if (client.connected()) client.publish(topic, msg);
  Serial.printf("[MQTT] %s => %s\n", topic, msg);
}

// ====== Servo điều khiển ======
void moveServoSmooth(Servo &s, int start, int end, int dur) {
  if (start == end) return;
  int step = (end > start) ? 1 : -1;
  int steps = abs(end - start);
  int dt = dur / steps;
  for (int p = start; p != end; p += step) {
    s.write(p);
    delay(dt);
  }
  s.write(end);
}

void setServo(Servo &s, bool &stateVar, bool open, const char* which, const char* source) {
  if (stateVar == open) return;

  int start = stateVar ? OPEN_ANGLE : CLOSED_ANGLE;
  int target = open ? OPEN_ANGLE : CLOSED_ANGLE;

  moveServoSmooth(s, start, target, BARRIER_SPEED_MS);
  stateVar = open;

  if (strcmp(which, "BARRIER-VAO") == 0 && !open) openedByRFID_Vao = false;
  if (strcmp(which, "BARRIER-RA") == 0 && !open) openedByRFID_Ra  = false;

  String msg = String(which) + (open ? " MỞ (" : " ĐÓNG (") + String(source) + ")";
  mqttPublish("status", msg.c_str());
  Serial.printf("[%s] %s bởi %s\n", which, open ? "MỞ" : "ĐÓNG", source);
}

// ====== Khởi tạo IR ======
void initTracker(IRTracker &t, int pin) {
  t.pin = pin;
  pinMode(pin, INPUT);
  t.lastRead = digitalRead(pin);
  t.stable = t.lastRead;
  t.lastChange = millis();
  t.sawLowAfterOpen = false;
}

// ====== Xử lý cảm biến trước ======
void handleBefore(IRTracker &t) {
  int r = digitalRead(t.pin);
  if (r != t.lastRead) {
    t.lastChange = millis();
    t.lastRead = r;
  }
  if (millis() - t.lastChange >= CONFIRM_MS) {
    t.stable = r;
  }
}

// ====== Xử lý cảm biến sau ======
void handleAfter(IRTracker &tAfter, bool &openedByRFID, const char* topicName, Servo &servo, bool &servoState, const char* which) {
  int r = digitalRead(tAfter.pin);
  if (r != tAfter.lastRead) {
    tAfter.lastChange = millis();
    tAfter.lastRead = r;
  }
  if (millis() - tAfter.lastChange >= CONFIRM_MS) {
    if (r != tAfter.stable) {
      tAfter.stable = r;
      String topic = String("parking/") + topicName;
      if (r == LOW) {
        mqttPublish(topic.c_str(), "CO XE");
        if (openedByRFID) tAfter.sawLowAfterOpen = true;
      } else {
        mqttPublish(topic.c_str(), "TRONG");
        if (openedByRFID && tAfter.sawLowAfterOpen) {
          setServo(servo, servoState, false, which, "IR_Passed");
          openedByRFID = false;
          tAfter.sawLowAfterOpen = false;
        }
      }
    }
  }
}

// ====== Parking slot ======
void handleSlots() {
  for (int i=0;i<4;i++) {
    int r = digitalRead(slots[i].pin);
    if (r != slots[i].lastRead) {
      slots[i].lastChange = millis();
      slots[i].lastRead = r;
    }
    if (millis() - slots[i].lastChange >= SLOT_CONFIRM_MS) {
      if (slots[i].stable != r) {
        slots[i].stable = r;
        String topic = String("parking/slot") + String(i+1);
        String payload = String("P") + String(i+1) + (r==LOW ? " CO XE" : " TRONG");
        mqttPublish(topic.c_str(), payload.c_str());
      }
    }
  }
}

// ====== RFID xử lý ======
String lastUid = "";
unsigned long lastTrigger = 0;
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  unsigned long now = millis();
  if (uid != lastUid || now - lastTrigger >= RFID_DEBOUNCE_MS) {
    lastUid = uid;
    lastTrigger = now;

    int vao = beforeVao.stable;
    int ra  = beforeRa.stable;

    if (vao == LOW && ra == HIGH) {
      if (!servoVaoState) {
        setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "RFID");
        openedByRFID_Vao = true;
        afterVao.sawLowAfterOpen = false;
      }
    } else if (ra == LOW && vao == HIGH) {
      if (!servoRaState) {
        setServo(servoRa, servoRaState, true, "BARRIER-RA", "RFID");
        openedByRFID_Ra = true;
        afterRa.sawLowAfterOpen = false;
      }
    } else {
      mqttPublish("status", "KHONG XAC DINH HUONG (VAO/RA)");
    }
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ====== MQTT callback ======
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[128];
  unsigned int copyLen = min(length, (unsigned int)sizeof(msg)-1);
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  if (strcmp(topic, "barrier") == 0) {
    if (strcmp(msg, "OPEN_VAO") == 0)
      setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "MQTT");
    else if (strcmp(msg, "CLOSE_VAO") == 0)
      setServo(servoVao, servoVaoState, false, "BARRIER-VAO", "MQTT");
    else if (strcmp(msg, "OPEN_RA") == 0)
      setServo(servoRa, servoRaState, true, "BARRIER-RA", "MQTT");
    else if (strcmp(msg, "CLOSE_RA") == 0)
      setServo(servoRa, servoRaState, false, "BARRIER-RA", "MQTT");
  }
}

// ====== MQTT reconnect ======
void reconnect() {
  while (!client.connected()) {
    String clientID = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      client.subscribe("barrier");
    } else delay(2000);
  }
}

// ====== setup ======
void setup() {
  Serial.begin(115200);
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
  rfid.PCD_Init();

  initTracker(beforeVao, IR_TRUOC_VAO);
  initTracker(afterVao, IR_SAU_VAO);
  initTracker(beforeRa, IR_TRUOC_RA);
  initTracker(afterRa, IR_SAU_RA);
  for (int i=0;i<4;i++) initTracker(slots[i], IR_PARK[i]);

  servoVao.attach(SERVO_VAO_PIN, 500, 2500);
  servoRa.attach(SERVO_RA_PIN, 500, 2500);
  servoVao.write(CLOSED_ANGLE);
  servoRa.write(CLOSED_ANGLE);

  WiFi.begin(ssid, password);
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  while (WiFi.status() != WL_CONNECTED) { delay(300); }
}

// ====== loop ======
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  handleBefore(beforeVao);
  handleBefore(beforeRa);
  handleRFID();
  handleAfter(afterVao, openedByRFID_Vao, "IR_SAU_VAO", servoVao, servoVaoState, "BARRIER-VAO");
  handleAfter(afterRa, openedByRFID_Ra, "IR_SAU_RA", servoRa, servoRaState, "BARRIER-RA");
  handleSlots();

  delay(5);
}
