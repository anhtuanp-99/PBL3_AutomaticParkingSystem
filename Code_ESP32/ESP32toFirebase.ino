#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <FirebaseESP32.h>
#include <time.h>

// ========== WIFI & FIREBASE CONFIG ==========
const char* ssid = "Kaka";
const char* password = "12345678";
#define FIREBASE_HOST "car01-351f6-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "snEXxxKZBN2MamzCNPH9B8sp0KTB3wP3nSzh0fvy"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ========== PIN DEFINITIONS ==========
// RFID Pins
const int RFID_SCK = 18, RFID_MISO = 19, RFID_MOSI = 23, RFID_SS = 5, RFID_RST = 2;

// Entry (VAO) Barrier
const int IR_VAO_1 = 15;  // Trước rào
const int IR_VAO_2 = 34;  // Sau rào
const int SERVO_VAO_PIN = 21;

// Exit (RA) Barrier
const int IR_RA_1 = 26;   // Trước rào
const int IR_RA_2 = 27;   // Sau rào
const int SERVO_RA_PIN = 25;

// Parking Slots
const int IR_PARK[4] = {13, 33, 14, 32};

// Servo Angles
const int SERVO_VAO_OPEN = 90, SERVO_VAO_CLOSED = 180;
const int SERVO_RA_OPEN = 90, SERVO_RA_CLOSED = 0;

// ========== TIMING CONSTANTS ==========
const unsigned long CONFIRM_MS = 120;           // Debounce cho IR
const unsigned long SLOT_CONFIRM_MS = 3000;     // Debounce cho parking slots
const unsigned long RFID_DEBOUNCE_MS = 3000;    // Ngăn quét trùng RFID
const unsigned long RECONNECT_INTERVAL = 30000; // Thử kết nối lại Firebase
const unsigned long BARRIER_TIMEOUT = 15000;    // Tự động đóng rào nếu quá lâu

// ========== OBJECTS ==========
MFRC522 rfid(RFID_SS, RFID_RST);
Servo servoVao, servoRa;

// ========== STATE TRACKING ==========
struct IRTracker {
  int pin;
  int lastRead;
  int stable;
  unsigned long lastChange;
  bool sawLowAfterOpen;
};

IRTracker irVao1, irVao2, irRa1, irRa2, slots[4];

bool servoVaoState = false;
bool servoRaState = false;
bool openedByRFID_Vao = false;
bool openedByRFID_Ra = false;
unsigned long barrierVaoOpenTime = 0;
unsigned long barrierRaOpenTime = 0;

String authorizedCarName = "";
String lastUid = "";
unsigned long lastTrigger = 0;

bool firebaseReady = false;
unsigned long lastReconnect = 0;

// ========== FIREBASE FUNCTIONS ==========
void initFirebase() {
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  config.timeout.serverResponse = 10 * 1000;
  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);
  
  int retries = 0;
  while (!Firebase.ready() && retries < 10) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  
  firebaseReady = Firebase.ready();
  Serial.println(firebaseReady ? "\n✅ Firebase OK" : "\n❌ Firebase Failed");
}

String getCurrentDate() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) return "unknown";
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d", &timeinfo);
  return String(buf);
}

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) return "00:00:00";
  char buf[10];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

unsigned long getCurrentTimestamp() {
  time_t now;
  time(&now);
  return (unsigned long)now;
}

void firebaseLog(String type, String message) {
  if (!firebaseReady) return;
  
  String date = getCurrentDate();
  if (date == "unknown") return;

  FirebaseJson json;
  json.set("type", type);
  json.set("message", message);
  json.set("time", getCurrentTime());

  String path = "/logs/" + date;
  if (Firebase.pushJSON(fbdo, path, json)) {
    Serial.printf("📝 Log: [%s] %s\n", type.c_str(), message.c_str());
  }
}

bool updateFirebaseState(String path, String value) {
  if (!firebaseReady) return false;
  
  if (Firebase.setString(fbdo, path, value)) {
    Serial.printf("✅ %s = %s\n", path.c_str(), value.c_str());
    return true;
  }
  return false;
}

// ========== TRANSACTION MANAGEMENT ==========
void startTransaction(String uid, String carName) {
  if (!firebaseReady) return;
  
  String path = "/sessions_active/" + uid;
  
  // Kiểm tra xem UID đã tồn tại chưa
  if (Firebase.get(fbdo, path) && fbdo.dataType() != "null") {
    firebaseLog("Warning", "UID đã tồn tại: " + uid);
    return;
  }

  FirebaseJson json;
  json.set("uid", uid);
  json.set("car_name", carName);
  json.set("time_in", getCurrentTime());
  json.set("date_in", getCurrentDate());
  json.set("timestamp_in", (int)getCurrentTimestamp());
  json.set("status", "IN_PROGRESS");

  if (Firebase.setJSON(fbdo, path, json)) {
    firebaseLog("Entry", carName + " (" + uid + ")");
  }
}

void completeTransaction(String uid) {
  if (!firebaseReady) return;
  
  String activePath = "/sessions_active/" + uid;

  if (!Firebase.getJSON(fbdo, activePath) || fbdo.dataType() == "null") {
    firebaseLog("Warning", "Không tìm thấy session: " + uid);
    return;
  }

  FirebaseJson sessionJson;
  sessionJson.setJsonData(fbdo.jsonString());
  
  FirebaseJsonData result;
  sessionJson.get(result, "timestamp_in");
  unsigned long timeIn = result.intValue;
  unsigned long timeOut = getCurrentTimestamp();
  long duration = timeOut - timeIn;
  
  // Format duration
  long days = duration / 86400;
  long hours = (duration % 86400) / 3600;
  long mins = (duration % 3600) / 60;
  long secs = duration % 60;
  char durationStr[50];
  sprintf(durationStr, "%ldd %ldh %ldm %lds", days, hours, mins, secs);
  
  sessionJson.set("time_out", getCurrentTime());
  sessionJson.set("date_out", getCurrentDate());
  sessionJson.set("timestamp_out", (int)timeOut);
  sessionJson.set("duration_sec", (int)duration);
  sessionJson.set("duration_str", String(durationStr));
  sessionJson.set("status", "COMPLETED");

  if (Firebase.pushJSON(fbdo, "/history_log", sessionJson)) {
    firebaseLog("Exit", "Duration: " + String(durationStr));
  }

  Firebase.deleteNode(fbdo, activePath);
}

// ========== AUTHORIZATION ==========
bool isUidAuthorized(String uid) {
  if (!firebaseReady) return false;
  
  String path = "/authorized_uids/" + uid;
  
  if (Firebase.getString(fbdo, path) && fbdo.dataType() != "null" && fbdo.stringData().length() > 0) {
    authorizedCarName = fbdo.stringData();
    firebaseLog("RFID", "✓ " + uid + " (" + authorizedCarName + ")");
    return true;
  }
  
  authorizedCarName = "Unauthorized";
  firebaseLog("RFID", "✗ " + uid);
  return false;
}

// ========== PARKING STATUS ==========
void updateTotalStatus() {
  int occupied = 0;
  for (int i = 0; i < 4; i++) {
    if (slots[i].stable == LOW) occupied++;
  }
  
  int free = 4 - occupied;
  updateFirebaseState("/parking/total_occupied", String(occupied));
  updateFirebaseState("/parking/total_free", String(free));
  Serial.printf("📊 %d/4 slots occupied\n", occupied);
}

// ========== SERVO CONTROL ==========
void setServo(Servo &s, bool &stateVar, bool open, const char* name, const char* reason, unsigned long &openTime) {
  if (stateVar == open) return;

  int target = open ? (strcmp(name, "VAO") == 0 ? SERVO_VAO_OPEN : SERVO_RA_OPEN) 
                    : (strcmp(name, "VAO") == 0 ? SERVO_VAO_CLOSED : SERVO_RA_CLOSED);
  
  s.write(target);
  stateVar = open;

  if (open) {
    openTime = millis();
  } else {
    // Reset flags khi đóng
    if (strcmp(name, "VAO") == 0) {
      openedByRFID_Vao = false;
      irVao2.sawLowAfterOpen = false;
    } else {
      openedByRFID_Ra = false;
      irRa2.sawLowAfterOpen = false;
    }
  }

  String msg = String(name) + (open ? " OPEN (" : " CLOSED (") + reason + ")";
  Serial.println("🚧 " + msg);
  firebaseLog("Barrier", msg);
  
  String path = "/parking/barrier" + String(name);
  updateFirebaseState(path, open ? "OPEN" : "CLOSED");
}

// ========== IR SENSOR HANDLING ==========
void initTracker(IRTracker &t, int pin) {
  pinMode(pin, (pin >= 32 && pin <= 39) ? INPUT : INPUT_PULLUP);
  t.pin = pin;
  t.lastRead = digitalRead(pin);
  t.stable = t.lastRead;
  t.lastChange = millis();
  t.sawLowAfterOpen = false;
}

void handleIRSensor(IRTracker &t, const char* fbPath, unsigned long debounceMs = CONFIRM_MS) {
  int r = digitalRead(t.pin);
  
  if (r != t.lastRead) {
    t.lastChange = millis();
    t.lastRead = r;
  }
  
  if (millis() - t.lastChange >= debounceMs && t.stable != r) {
    t.stable = r;
    String status = (r == LOW) ? "CO_XE" : "TRONG";
    updateFirebaseState(String("/parking/") + fbPath, status);
  }
}

void handleBarrierExit(IRTracker &tAfter, bool &openedByRFID, const char* fbPath, 
                       Servo &servo, bool &servoState, const char* name, unsigned long &openTime) {
  int r = digitalRead(tAfter.pin);
  
  if (r != tAfter.lastRead) {
    tAfter.lastChange = millis();
    tAfter.lastRead = r;
  }
  
  if (millis() - tAfter.lastChange >= CONFIRM_MS && r != tAfter.stable) {
    tAfter.stable = r;
    String status = (r == LOW) ? "CO_XE" : "TRONG";
    updateFirebaseState(String("/parking/") + fbPath, status);
    
    if (r == LOW && openedByRFID && servoState) {
      tAfter.sawLowAfterOpen = true;
      Serial.printf("✓ Xe đi qua %s\n", name);
    } 
    else if (r == HIGH && servoState && openedByRFID && tAfter.sawLowAfterOpen) {
      Serial.printf("🔒 Đóng %s\n", name);
      setServo(servo, servoState, false, name, "Auto", openTime);
    }
  }
}

void handleSlots() {
  bool changed = false;
  
  for (int i = 0; i < 4; i++) {
    int r = digitalRead(slots[i].pin);
    
    if (r != slots[i].lastRead) {
      slots[i].lastChange = millis();
      slots[i].lastRead = r;
    }
    
    if (millis() - slots[i].lastChange >= SLOT_CONFIRM_MS && slots[i].stable != r) {
      slots[i].stable = r;
      String status = (r == LOW) ? "CO_XE" : "TRONG";
      updateFirebaseState("/parking/slot" + String(i + 1), status);
      Serial.printf("🅿️ Slot %d: %s\n", i + 1, status.c_str());
      changed = true;
    }
  }
  
  if (changed) updateTotalStatus();
}

// ========== RFID HANDLING ==========
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  unsigned long now = millis();
  if (uid == lastUid && (now - lastTrigger < RFID_DEBOUNCE_MS)) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }
  
  lastUid = uid;
  lastTrigger = now;
  
  Serial.printf("📇 RFID: %s\n", uid.c_str());

  if (!isUidAuthorized(uid)) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }

  int vao = irVao1.stable;
  int ra = irRa1.stable;

  // Logic xác định hướng
  if (vao == LOW && ra == HIGH) {
    // Xe ở cổng VÀO
    if (!servoVaoState) {
      Serial.println("➡️ Mở rào VÀO");
      setServo(servoVao, servoVaoState, true, "VAO", "RFID", barrierVaoOpenTime);
      openedByRFID_Vao = true;
      irVao2.sawLowAfterOpen = false;
      startTransaction(uid, authorizedCarName);
    }
  } 
  else if (ra == LOW && vao == HIGH) {
    // Xe ở cổng RA
    if (!servoRaState) {
      Serial.println("➡️ Mở rào RA");
      setServo(servoRa, servoRaState, true, "RA", "RFID", barrierRaOpenTime);
      openedByRFID_Ra = true;
      irRa2.sawLowAfterOpen = false;
      completeTransaction(uid);
    }
  } 
  else {
    Serial.printf("❌ Hướng không xác định: VAO=%d, RA=%d\n", vao, ra);
    firebaseLog("Error", "Invalid direction");
  }
  
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ========== BARRIER TIMEOUT ==========
void checkBarrierTimeout() {
  unsigned long now = millis();
  
  // Timeout cho rào VÀO
  if (servoVaoState && openedByRFID_Vao && (now - barrierVaoOpenTime > BARRIER_TIMEOUT)) {
    Serial.println("⚠️ TIMEOUT - Đóng rào VÀO");
    setServo(servoVao, servoVaoState, false, "VAO", "Timeout", barrierVaoOpenTime);
  }
  
  // Timeout cho rào RA
  if (servoRaState && openedByRFID_Ra && (now - barrierRaOpenTime > BARRIER_TIMEOUT)) {
    Serial.println("⚠️ TIMEOUT - Đóng rào RA");
    setServo(servoRa, servoRaState, false, "RA", "Timeout", barrierRaOpenTime);
  }
}

// ========== FIREBASE RECONNECT ==========
void checkFirebaseConnection() {
  if (!firebaseReady && (millis() - lastReconnect > RECONNECT_INTERVAL)) {
    Serial.println("🔄 Reconnecting Firebase...");
    initFirebase();
    lastReconnect = millis();
  }
}

// ========== INIT FIREBASE NODES ==========
void initFirebaseNodes() {
  if (!firebaseReady) return;
  
  Serial.println("⚙️ Init Firebase nodes...");

  // Init authorized UIDs
  if (!Firebase.get(fbdo, "/authorized_uids")) {
    Firebase.setString(fbdo, "/authorized_uids/sample", "Sample_User");
  }

  // Init parking status
  updateFirebaseState("/parking/total_occupied", "0");
  updateFirebaseState("/parking/total_free", "4");
  updateFirebaseState("/parking/barrierVAO", "CLOSED");
  updateFirebaseState("/parking/barrierRA", "CLOSED");
  
  // Init IR status
  updateFirebaseState("/parking/IR_VAO_1", "TRONG");
  updateFirebaseState("/parking/IR_VAO_2", "TRONG");
  updateFirebaseState("/parking/IR_RA_1", "TRONG");
  updateFirebaseState("/parking/IR_RA_2", "TRONG");
  
  // Init slots
  for (int i = 1; i <= 4; i++) {
    updateFirebaseState("/parking/slot" + String(i), "TRONG");
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SMART PARKING SYSTEM ===");

  // Init RFID
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
  rfid.PCD_Init();
  Serial.println("✅ RFID OK");

  // Init IR Sensors
  initTracker(irVao1, IR_VAO_1);
  initTracker(irVao2, IR_VAO_2);
  initTracker(irRa1, IR_RA_1);
  initTracker(irRa2, IR_RA_2);
  for (int i = 0; i < 4; i++) initTracker(slots[i], IR_PARK[i]);
  Serial.println("✅ IR Sensors OK");
  
  // Init Servos
  servoVao.attach(SERVO_VAO_PIN, 500, 2500);
  servoRa.attach(SERVO_RA_PIN, 500, 2500);
  servoVao.write(SERVO_VAO_CLOSED);
  servoRa.write(SERVO_RA_CLOSED);
  Serial.println("✅ Servos OK");

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("🌐 WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ WiFi OK - IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi Failed");
    return;
  }

  // Sync NTP
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("⏰ NTP syncing...");
  delay(2000);

  // Init Firebase
  initFirebase();
  if (firebaseReady) {
    initFirebaseNodes();
    updateTotalStatus();
  }

  Serial.println("\n=== SYSTEM READY ===\n");
}

// ========== MAIN LOOP ==========
void loop() {
  checkFirebaseConnection();
  
  // Handle IR sensors
  handleIRSensor(irVao1, "IR_VAO_1");
  handleIRSensor(irRa1, "IR_RA_1");
  
  handleBarrierExit(irVao2, openedByRFID_Vao, "IR_VAO_2", servoVao, servoVaoState, "VAO", barrierVaoOpenTime);
  handleBarrierExit(irRa2, openedByRFID_Ra, "IR_RA_2", servoRa, servoRaState, "RA", barrierRaOpenTime);
  
  handleSlots();
  handleRFID();
  
  // Check timeout
  checkBarrierTimeout();
  
  delay(10);
}