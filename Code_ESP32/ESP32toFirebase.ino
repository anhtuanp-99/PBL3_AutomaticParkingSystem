#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <FirebaseESP32.h>
#include <time.h>

// ====== Cấu hình WiFi ======
const char* ssid = "Kaka";
const char* password = "12345678";

// ====== Cấu hình Firebase ======
#define FIREBASE_HOST "car01-351f6-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "snEXxxKZBN2MamzCNPH9B8sp0KTB3wP3nSzh0fvy"

// --- Đối tượng Firebase ---
FirebaseData fbdo;
FirebaseData streamData;
FirebaseAuth auth;
FirebaseConfig config;

// ====== PIN mapping ======
const int RFID_SCK = 18, RFID_MISO = 19, RFID_MOSI = 23, RFID_SS = 5, RFID_RST = 2;

// Barrier VÀO
const int IR_VAO_1 = 15;
const int IR_VAO_2 = 34;
const int SERVO_VAO_PIN = 21;

// Barrier RA
const int IR_RA_1 = 26;
const int IR_RA_2 = 27;
const int SERVO_RA_PIN = 25;

// Parking slots
const int IR_PARK[4] = {13, 33, 14, 32};

// Servo góc - Tách riêng cho VÀO và RA
const int SERVO_VAO_OPEN = 90;
const int SERVO_VAO_CLOSED = 180;

const int SERVO_RA_OPEN = 90;
const int SERVO_RA_CLOSED = 0;

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
const unsigned long AUTO_CLOSE_TIMEOUT = 15000; // 15 giây tự động đóng nếu không có xe đi qua

// ====== Cấu trúc theo dõi IR ======
struct IRTracker {
  int pin;
  int lastRead; 
  int stable;   
  unsigned long lastChange; 
  bool sawLowAfterOpen;
};

IRTracker irVao1, irVao2, irRa1, irRa2, slots[4];

// ====== Trạng thái barrier ======
bool openedByRFID_Vao = false;
bool openedByRFID_Ra = false;
unsigned long barrierVaoOpenTime = 0;  // Thời điểm mở rào VÀO
unsigned long barrierRaOpenTime = 0;   // Thời điểm mở rào RA

String authorizedCarName = "";
String lastUid = ""; 
unsigned long lastTrigger = 0;

// Biến kiểm soát kết nối
bool firebaseReady = false;
unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 30000;

// ===========================================
// ====== CÁC HÀM TRỢ GIÚP FIREBASE ======
// ===========================================

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
  
  if (Firebase.ready()) {
    firebaseReady = true;
    Serial.println("\n✅ Kết nối Firebase thành công!");
  } else {
    firebaseReady = false;
    Serial.println("\n❌ Không thể kết nối Firebase!");
  }
}

String getCurrentDate() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) return "unknown_date"; 
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d", &timeinfo);
  return String(buffer);
}

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) return "00:00:00";
  char buffer[10];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
  return String(buffer);
}

unsigned long getCurrentTimestamp() {
  time_t now;
  time(&now);
  return (unsigned long)now;
}

void firebaseLog(String type, String message) {
  if (!firebaseReady) return;
  
  String date = getCurrentDate();
  if (date == "unknown_date") return;

  String timeNow = getCurrentTime();
  String path = "/logs/" + date; 

  FirebaseJson json;
  json.set("type", type);
  json.set("message", message);
  json.set("time", timeNow);

  if (Firebase.pushJSON(fbdo, path, json)) {
    Serial.printf("✅ FB Log: [%s] %s\n", type.c_str(), message.c_str());
  } else {
    Serial.println("❌ Lỗi log Firebase: " + fbdo.errorReason());
  }
}

void updateFirebaseState(String path, String value) {
  if (!firebaseReady) return;
  
  if (Firebase.setString(fbdo, path, value)) {
    Serial.printf("✅ FB State: %s -> %s\n", path.c_str(), value.c_str());
  } else {
    Serial.println("❌ Lỗi cập nhật state Firebase: " + fbdo.errorReason());
  }
}

// =======================================================
// ====== LOGIC LƯU TRỮ DỮ LIỆU GIAO DỊCH ======
// =======================================================

void startTransaction(String uid, String carName) {
  if (!firebaseReady) return;
  
  String path = "/sessions_active/" + uid;
  
  if (Firebase.get(fbdo, path)) {
    if (fbdo.dataType() != "null") {
      firebaseLog("Transaction", "UID đã tồn tại trong sessions_active.");
      return;
    }
  }

  FirebaseJson json;
  json.set("uid", uid);
  json.set("car_name", carName);
  json.set("time_in_str", getCurrentTime());
  json.set("date_in_str", getCurrentDate());
  json.set("timestamp_in", (int)getCurrentTimestamp());
  json.set("status", "IN_PROGRESS");

  if (Firebase.setJSON(fbdo, path, json)) {
    firebaseLog("Transaction", "Tạo mới giao dịch cho UID: " + uid);
  } else {
    Serial.println("❌ Lỗi setJSON cho sessions_active: " + fbdo.errorReason());
  }
}

void completeTransaction(String uid) {
  if (!firebaseReady) return;
  
  String activePath = "/sessions_active/" + uid;

  if (!Firebase.getJSON(fbdo, activePath)) {
    firebaseLog("Transaction", "Không tìm thấy session active cho UID: " + uid);
    return;
  }

  if (fbdo.dataType() == "null") {
    firebaseLog("Transaction", "Session đã bị xóa hoặc không tồn tại.");
    return;
  }

  FirebaseJson sessionJson;
  sessionJson.setJsonData(fbdo.jsonString()); 
  
  FirebaseJsonData result;
  sessionJson.get(result, "timestamp_in");
  unsigned long timeIn = result.intValue;

  unsigned long timeOut = getCurrentTimestamp();
  long durationSec = timeOut - timeIn;
  
  long days = durationSec / (3600 * 24);
  long hours = (durationSec % (3600 * 24)) / 3600;
  long mins = (durationSec % 3600) / 60;
  long secs = durationSec % 60;
  
  String durationStr = String(days) + "d " + String(hours) + "h " + String(mins) + "m " + String(secs) + "s";
  
  sessionJson.set("time_out_str", getCurrentTime());
  sessionJson.set("date_out_str", getCurrentDate());
  sessionJson.set("timestamp_out", (int)timeOut);
  sessionJson.set("duration_sec", (int)durationSec);
  sessionJson.set("duration_str", durationStr);
  sessionJson.set("status", "COMPLETED");

  if (Firebase.pushJSON(fbdo, "/history_log", sessionJson)) {
    firebaseLog("Transaction", "Hoàn tất. Thời gian: " + durationStr);
  } else {
    Serial.println("❌ Lỗi push JSON vào history_log: " + fbdo.errorReason());
  }

  if (Firebase.deleteNode(fbdo, activePath)) {
    Serial.println("✅ Xóa session active thành công.");
  } else {
    Serial.println("❌ Lỗi xóa session active: " + fbdo.errorReason());
  }
}

// =======================================================
// ====== LOGIC ĐIỀU KHIỂN VÀ CẢM BIẾN ======
// =======================================================

void streamCallback(StreamData data) {
  if (data.dataType() == "string") {
    String cmd = data.stringData();
    String path = data.dataPath();

    Firebase.setString(fbdo, path, "NONE");

    if (path == "/commands/barrierVaoControl") {
      if (cmd == "OPEN") {
        setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "Manual");
        openedByRFID_Vao = false; // Manual không theo dõi tự động đóng
      } else if (cmd == "CLOSE") {
        setServo(servoVao, servoVaoState, false, "BARRIER-VAO", "Manual");
        openedByRFID_Vao = false;
      }
    } 
    else if (path == "/commands/barrierRaControl") {
      if (cmd == "OPEN") {
        setServo(servoRa, servoRaState, true, "BARRIER-RA", "Manual");
        openedByRFID_Ra = false;
      } else if (cmd == "CLOSE") {
        setServo(servoRa, servoRaState, false, "BARRIER-RA", "Manual");
        openedByRFID_Ra = false;
      }
    }
  }
}

void streamTimeoutCallback(bool timeout) {
  if (timeout) {
    Serial.println("⚠️ Stream timeout, đang kết nối lại...");
    firebaseReady = false;
  }
}

bool isUidAuthorized(String uid) {
  if (!firebaseReady) return false;
  
  String path = "/authorized_uids/" + uid;
  
  if (Firebase.getString(fbdo, path)) {
    if (fbdo.dataType() != "null" && fbdo.stringData().length() > 0) {
      authorizedCarName = fbdo.stringData();
      firebaseLog("RFIDAccess", "Thẻ hợp lệ: " + uid + " (" + authorizedCarName + ")");
      return true;
    }
  }
  
  authorizedCarName = "Khách_Vãng_Lai";
  firebaseLog("RFIDError", "Thẻ không hợp lệ: " + uid);
  return false;
}

void updateTotalStatus() {
  int occupied = 0;
  int totalSlots = 4;
  
  for (int i = 0; i < totalSlots; i++) {
    if (slots[i].stable == LOW) {
      occupied++;
    }
  }
  
  int free = totalSlots - occupied;
  
  updateFirebaseState("/parking/total_occupied", String(occupied));
  updateFirebaseState("/parking/total_free", String(free));
  Serial.printf("Tổng quan: %d chiếm, %d trống.\n", occupied, free);
}

void setServo(Servo &s, bool &stateVar, bool open, const char* which, const char* source) {
  if (stateVar == open) return;

  int targetOpen, targetClosed;
  if (strcmp(which, "BARRIER-VAO") == 0) {
    targetOpen = SERVO_VAO_OPEN;
    targetClosed = SERVO_VAO_CLOSED;
  } else {
    targetOpen = SERVO_RA_OPEN;
    targetClosed = SERVO_RA_CLOSED;
  }

  int target = open ? targetOpen : targetClosed;
  s.write(target);
  stateVar = open;

  if (strcmp(which, "BARRIER-VAO") == 0 && !open) {
    openedByRFID_Vao = false;
    irVao2.sawLowAfterOpen = false;
    barrierVaoOpenTime = 0;
  }
  if (strcmp(which, "BARRIER-RA") == 0 && !open) {
    openedByRFID_Ra = false;
    irRa2.sawLowAfterOpen = false;
    barrierRaOpenTime = 0;
  }

  String msg = String(which) + (open ? " MỞ (" : " ĐÓNG (") + String(source) + ")";
  firebaseLog("Barrier", msg); 
  String statePath = (strcmp(which, "BARRIER-VAO") == 0) ? "/parking/barrierVao" : "/parking/barrierRa";
  updateFirebaseState(statePath, open ? "OPEN" : "CLOSED");
}

void initTracker(IRTracker &t, int pin) {
  if (pin >= 32 && pin <= 39) {
    pinMode(pin, INPUT);
    Serial.printf("Pin %d là INPUT_ONLY.\n", pin);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
  
  t.pin = pin;
  t.lastRead = digitalRead(pin);
  t.stable = t.lastRead;
  t.lastChange = millis();
  t.sawLowAfterOpen = false;
}

void handleBefore(IRTracker &t) {
  int r = digitalRead(t.pin);
  if (r != t.lastRead) {
    t.lastChange = millis();
    t.lastRead = r;
  }
  
  if (millis() - t.lastChange >= CONFIRM_MS) {
    if (t.stable != r) {
      t.stable = r;
      Serial.printf("🔔 IR Pin %d thay đổi: %s\n", t.pin, r == LOW ? "LOW(Có xe)" : "HIGH(Trống)");
    }
  }
}

void handleAfter(IRTracker &tAfter, bool &openedByRFID, const char* topicName, Servo &servo, bool &servoState, const char* which) {
  int r = digitalRead(tAfter.pin);
  
  if (r != tAfter.lastRead) {
    tAfter.lastChange = millis();
    tAfter.lastRead = r;
  }
  
  if (millis() - tAfter.lastChange >= CONFIRM_MS && r != tAfter.stable) {
    tAfter.stable = r;
    String topic = String("parking/") + topicName; 
    
    if (r == LOW) {
      Serial.printf("🚗 Xe đã đến cảm biến SAU (%s)\n", topicName);
      updateFirebaseState(topic, "CO XE");
      
      if (openedByRFID) {
        tAfter.sawLowAfterOpen = true;
        Serial.printf("✓ Đã đánh dấu xe đi qua %s\n", which);
      }
    } else {
      Serial.printf("✓ Xe đã rời cảm biến SAU (%s)\n", topicName);
      updateFirebaseState(topic, "TRONG");
      
      // CHỈ ĐÓNG KHI: Rào được mở bởi RFID VÀ xe đã đi qua điểm giữa
      if (openedByRFID && tAfter.sawLowAfterOpen) {
        Serial.printf("🔒 Đóng %s - Xe đã đi qua hoàn toàn\n", which);
        setServo(servo, servoState, false, which, "IR_Passed");
      }
    }
  }
}

void handleSlots() {
  bool stateChanged = false;
  
  for (int i = 0; i < 4; i++) {
    int r = digitalRead(slots[i].pin);
    if (r != slots[i].lastRead) {
      slots[i].lastChange = millis();
      slots[i].lastRead = r;
    }
    
    if (millis() - slots[i].lastChange >= SLOT_CONFIRM_MS) {
      if (slots[i].stable != r) {
        slots[i].stable = r;
        String topic = String("parking/slot") + String(i + 1);
        String payload = (r == LOW ? "CO XE" : "TRONG"); 
        
        updateFirebaseState(topic, payload);
        stateChanged = true; 
      }
    }
  }
  
  if (stateChanged) {
    updateTotalStatus();
  }
}

void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

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
  
  Serial.printf("RFID Quét: %s\n", uid.c_str());
  firebaseLog("RFIDScan", uid); 

  if (isUidAuthorized(uid)) {
    int vao = irVao1.stable;
    int ra = irRa1.stable;

    Serial.printf("🔍 DEBUG - IR_VAO_1: %s, IR_RA_1: %s\n", 
                  vao == LOW ? "LOW(Có xe)" : "HIGH(Trống)", 
                  ra == LOW ? "LOW(Có xe)" : "HIGH(Trống)");

    if (vao == LOW && ra == HIGH) {
      Serial.println("✅ Điều kiện VÀO: IR_VAO_1=LOW && IR_RA_1=HIGH");
      if (!servoVaoState) {
        Serial.println("➡️ Mở rào VÀO...");
        setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "RFID");
        openedByRFID_Vao = true;
        irVao2.sawLowAfterOpen = false;
        barrierVaoOpenTime = millis(); // Ghi lại thời điểm mở
        startTransaction(uid, authorizedCarName);
      } else {
        Serial.println("⚠️ Rào VÀO đã mở rồi!");
      }
    } else if (ra == LOW && vao == HIGH) {
      Serial.println("✅ Điều kiện RA: IR_RA_1=LOW && IR_VAO_1=HIGH");
      if (!servoRaState) {
        Serial.println("➡️ Mở rào RA...");
        setServo(servoRa, servoRaState, true, "BARRIER-RA", "RFID");
        openedByRFID_Ra = true;
        irRa2.sawLowAfterOpen = false;
        barrierRaOpenTime = millis(); // Ghi lại thời điểm mở
        completeTransaction(uid);
      } else {
        Serial.println("⚠️ Rào RA đã mở rồi!");
      }
    } else {
      Serial.printf("❌ HƯỚNG KHÔNG XÁC ĐỊNH - vao=%d, ra=%d\n", vao, ra);
      firebaseLog("RFIDError", "HƯỚNG KHÔNG XÁC ĐỊNH - VAO:" + String(vao) + " RA:" + String(ra));
    }
  } 
  
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ====== HÀM KIỂM TRA TỰ ĐỘNG ĐÓNG RÀO ======
void checkAutoClose() {
  unsigned long now = millis();
  
  // Kiểm tra Barrier VÀO
  if (openedByRFID_Vao && servoVaoState && barrierVaoOpenTime > 0) {
    if (now - barrierVaoOpenTime > AUTO_CLOSE_TIMEOUT) {
      Serial.println("⏰ Timeout! Tự động đóng rào VÀO sau 15 giây");
      setServo(servoVao, servoVaoState, false, "BARRIER-VAO", "Timeout");
    }
  }
  
  // Kiểm tra Barrier RA
  if (openedByRFID_Ra && servoRaState && barrierRaOpenTime > 0) {
    if (now - barrierRaOpenTime > AUTO_CLOSE_TIMEOUT) {
      Serial.println("⏰ Timeout! Tự động đóng rào RA sau 15 giây");
      setServo(servoRa, servoRaState, false, "BARRIER-RA", "Timeout");
    }
  }
}

void initFirebaseNodes() {
  if (!firebaseReady) return;
  
  Serial.println("⚙️ Khởi tạo node Firebase...");

  String commandsPath = "/commands";
  if (!Firebase.get(fbdo, commandsPath)) { 
    FirebaseJson json;
    json.set("barrierVaoControl", "NONE"); 
    json.set("barrierRaControl", "NONE");
    if (Firebase.setJSON(fbdo, commandsPath, json)) {
      Serial.println("✅ Tạo /commands");
    }
  }
  
  String uidsPath = "/authorized_uids";
  if (!Firebase.get(fbdo, uidsPath)) { 
    if (Firebase.setString(fbdo, uidsPath + "/00000000", "Sample_User")) {
      Serial.println("✅ Tạo /authorized_uids");
    }
  }

  String parkingPath = "/parking";
  if (!Firebase.get(fbdo, parkingPath + "/total_occupied")) {
    updateFirebaseState(parkingPath + "/total_occupied", "0");
    updateFirebaseState(parkingPath + "/total_free", "4");
    Serial.println("✅ Tạo trạng thái đỗ xe");
  }
}

void checkFirebaseConnection() {
  if (!firebaseReady) {
    unsigned long now = millis();
    if (now - lastReconnect > RECONNECT_INTERVAL) {
      Serial.println("🔄 Thử kết nối lại Firebase...");
      initFirebase();
      if (firebaseReady) {
        initFirebaseNodes();
        
        if (Firebase.beginStream(streamData, "/commands")) {
          Firebase.setStreamCallback(streamData, streamCallback, streamTimeoutCallback);
          Serial.println("✅ Stream đã được khởi động lại");
        }
      }
      lastReconnect = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== KHỞI ĐỘNG HỆ THỐNG ===");

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
  rfid.PCD_Init();
  Serial.println("✅ RFID Initialized");

  initTracker(irVao1, IR_VAO_1);
  initTracker(irVao2, IR_VAO_2);
  initTracker(irRa1, IR_RA_1);
  initTracker(irRa2, IR_RA_2);
  for (int i = 0; i < 4; i++) initTracker(slots[i], IR_PARK[i]);
  Serial.println("✅ IR Trackers Initialized");
  
  servoVao.attach(SERVO_VAO_PIN, 500, 2500);
  servoRa.attach(SERVO_RA_PIN, 500, 2500);
  servoVao.write(SERVO_VAO_CLOSED);
  servoRa.write(SERVO_RA_CLOSED);
  Serial.println("✅ Servos Initialized");

  WiFi.begin(ssid, password);
  Serial.print("Kết nối WiFi");
  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 20) {
    delay(500);
    Serial.print(".");
    wifiRetries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi đã kết nối!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Không thể kết nối WiFi!");
    return;
  }

  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("⏰ Đồng bộ NTP...");
  delay(2000);

  initFirebase();
  if (firebaseReady) {
    initFirebaseNodes();
    
    if (Firebase.beginStream(streamData, "/commands")) {
      Firebase.setStreamCallback(streamData, streamCallback, streamTimeoutCallback);
      Serial.println("✅ Stream đã được khởi động");
    } else {
      Serial.println("❌ Lỗi khởi động stream: " + streamData.errorReason());
    }
    
    updateTotalStatus();
  }

  Serial.println("=== HỆ THỐNG SẴN SÀNG ===\n");
}

void loop() {
  checkFirebaseConnection();
  
  handleBefore(irVao1);
  handleBefore(irRa1);
  handleAfter(irVao2, openedByRFID_Vao, "IR_VAO_2", servoVao, servoVaoState, "BARRIER-VAO"); 
  handleAfter(irRa2, openedByRFID_Ra, "IR_RA_2", servoRa, servoRaState, "BARRIER-RA");
  handleSlots();
  handleRFID();
  
  // QUAN TRỌNG: Kiểm tra tự động đóng
  checkAutoClose();
  
  delay(10);
}