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
  Serial.printf("📊 Tổng quan: %d chiếm, %d trống.\n", occupied, free);
}

void setServo(Servo &s, bool &stateVar, bool open, const char* which, const char* source) {
  if (stateVar == open) {
    Serial.printf("⚠️ %s đã ở trạng thái %s rồi!\n", which, open ? "MỞ" : "ĐÓNG");
    return;
  }

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

  // Reset flags khi đóng barrier
  if (!open) {
    if (strcmp(which, "BARRIER-VAO") == 0) {
      openedByRFID_Vao = false;
      irVao2.sawLowAfterOpen = false;
    } else {
      openedByRFID_Ra = false;
      irRa2.sawLowAfterOpen = false;
    }
  }

  String msg = String(which) + (open ? " MỞ (" : " ĐÓNG (") + String(source) + ")";
  Serial.println("🚧 " + msg);
  firebaseLog("Barrier", msg); 
  
  String statePath = (strcmp(which, "BARRIER-VAO") == 0) ? "/parking/barrierVao" : "/parking/barrierRa";
  updateFirebaseState(statePath, open ? "OPEN" : "CLOSED");
}

void initTracker(IRTracker &t, int pin) {
  if (pin >= 32 && pin <= 39) {
    pinMode(pin, INPUT);
    Serial.printf("📌 Pin %d là INPUT_ONLY.\n", pin);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
  
  t.pin = pin;
  t.lastRead = digitalRead(pin);
  t.stable = t.lastRead;
  t.lastChange = millis();
  t.sawLowAfterOpen = false;
}

// Hàm xử lý IR1 (cảm biến TRƯỚC rào) - CẬP NHẬT LÊN FIREBASE
void handleBefore(IRTracker &t, const char* topicName) {
  int r = digitalRead(t.pin);
  if (r != t.lastRead) {
    t.lastChange = millis();
    t.lastRead = r;
  }
  
  if (millis() - t.lastChange >= CONFIRM_MS) {
    if (t.stable != r) {
      t.stable = r;
      Serial.printf("🔔 IR Pin %d (%s) thay đổi: %s\n", t.pin, topicName, r == LOW ? "LOW(Có xe)" : "HIGH(Trống)");
      
      // CẬP NHẬT LÊN FIREBASE
      String topic = String("parking/") + topicName;
      String payload = (r == LOW ? "CO XE" : "TRONG");
      updateFirebaseState(topic, payload);
    }
  }
}

// Hàm xử lý IR2 (cảm biến SAU rào) - Có logic đóng rào
void handleAfter(IRTracker &tAfter, bool &openedByRFID, const char* topicName, Servo &servo, bool &servoState, const char* which) {
  int r = digitalRead(tAfter.pin);
  
  if (r != tAfter.lastRead) {
    tAfter.lastChange = millis();
    tAfter.lastRead = r;
  }
  
  // Chỉ xử lý khi trạng thái ổn định
  if (millis() - tAfter.lastChange >= CONFIRM_MS && r != tAfter.stable) {
    tAfter.stable = r;
    String topic = String("parking/") + topicName; 
    
    if (r == LOW) {
      // Xe đến cảm biến sau
      Serial.printf("🚗 Xe đã đến cảm biến SAU (%s)\n", topicName);
      updateFirebaseState(topic, "CO XE");
      
      // Đánh dấu xe đã đi qua nếu barrier được mở bởi RFID
      if (openedByRFID && servoState) {
        tAfter.sawLowAfterOpen = true;
        Serial.printf("✓ Đã đánh dấu xe đi qua %s\n", which);
      }
    } else {
      // Xe rời khỏi cảm biến sau
      Serial.printf("✅ Xe đã rời cảm biến SAU (%s)\n", topicName);
      updateFirebaseState(topic, "TRONG");
      
      // CHỈ ĐÓNG KHI: Barrier đang mở VÀ đã được đánh dấu xe đi qua
      if (servoState && openedByRFID && tAfter.sawLowAfterOpen) {
        Serial.printf("🔒 Đóng %s - Xe đã đi qua hoàn toàn\n", which);
        setServo(servo, servoState, false, which, "IR_AutoClose");
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
        
        Serial.printf("🅿️ Slot %d: %s\n", i + 1, payload.c_str());
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
  
  Serial.printf("📇 RFID Quét: %s\n", uid.c_str());
  firebaseLog("RFIDScan", uid); 

  if (isUidAuthorized(uid)) {
    int vao = irVao1.stable;
    int ra = irRa1.stable;

    Serial.printf("🔍 DEBUG - IR_VAO_1: %s, IR_RA_1: %s\n", 
                  vao == LOW ? "LOW(Có xe)" : "HIGH(Trống)", 
                  ra == LOW ? "LOW(Có xe)" : "HIGH(Trống)");

    // Logic xác định hướng: VÀO hay RA
    if (vao == LOW && ra == HIGH) {
      Serial.println("✅ Điều kiện VÀO: IR_VAO_1=LOW && IR_RA_1=HIGH");
      if (!servoVaoState) {
        Serial.println("➡️ Mở rào VÀO...");
        setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "RFID");
        openedByRFID_Vao = true;
        irVao2.sawLowAfterOpen = false;
        startTransaction(uid, authorizedCarName);
      } else {
        Serial.println("⚠️ Rào VÀO đã mở rồi!");
      }
    } 
    else if (ra == LOW && vao == HIGH) {
      Serial.println("✅ Điều kiện RA: IR_RA_1=LOW && IR_VAO_1=HIGH");
      if (!servoRaState) {
        Serial.println("➡️ Mở rào RA...");
        setServo(servoRa, servoRaState, true, "BARRIER-RA", "RFID");
        openedByRFID_Ra = true;
        irRa2.sawLowAfterOpen = false;
        completeTransaction(uid);
      } else {
        Serial.println("⚠️ Rào RA đã mở rồi!");
      }
    } 
    else {
      Serial.printf("❌ HƯỚNG KHÔNG XÁC ĐỊNH - VAO:%d, RA:%d\n", vao, ra);
      firebaseLog("RFIDError", "HƯỚNG KHÔNG XÁC ĐỊNH - VAO:" + String(vao) + " RA:" + String(ra));
    }
  }
  
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void initFirebaseNodes() {
  if (!firebaseReady) return;
  
  Serial.println("⚙️ Khởi tạo node Firebase...");

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
    updateFirebaseState(parkingPath + "/barrierVao", "CLOSED");
    updateFirebaseState(parkingPath + "/barrierRa", "CLOSED");
    
    // Khởi tạo trạng thái IR1 và IR2
    updateFirebaseState(parkingPath + "/IR_VAO_1", "TRONG");
    updateFirebaseState(parkingPath + "/IR_VAO_2", "TRONG");
    updateFirebaseState(parkingPath + "/IR_RA_1", "TRONG");
    updateFirebaseState(parkingPath + "/IR_RA_2", "TRONG");
    
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
      }
      lastReconnect = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== KHỞI ĐỘNG HỆ THỐNG PARKING TỰ ĐỘNG ===");

  // Khởi tạo RFID
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
  rfid.PCD_Init();
  Serial.println("✅ RFID Initialized");

  // Khởi tạo IR Sensors
  initTracker(irVao1, IR_VAO_1);
  initTracker(irVao2, IR_VAO_2);
  initTracker(irRa1, IR_RA_1);
  initTracker(irRa2, IR_RA_2);
  for (int i = 0; i < 4; i++) initTracker(slots[i], IR_PARK[i]);
  Serial.println("✅ IR Trackers Initialized");
  
  // Khởi tạo Servos
  servoVao.attach(SERVO_VAO_PIN, 500, 2500);
  servoRa.attach(SERVO_RA_PIN, 500, 2500);
  servoVao.write(SERVO_VAO_CLOSED);
  servoRa.write(SERVO_RA_CLOSED);
  Serial.println("✅ Servos Initialized (Đã đóng)");

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.print("🌐 Kết nối WiFi");
  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 20) {
    delay(500);
    Serial.print(".");
    wifiRetries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi đã kết nối!");
    Serial.print("📍 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Không thể kết nối WiFi!");
    return;
  }

  // Đồng bộ thời gian NTP
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("⏰ Đồng bộ NTP...");
  delay(2000);

  // Khởi tạo Firebase
  initFirebase();
  if (firebaseReady) {
    initFirebaseNodes();
    updateTotalStatus();
  }

  Serial.println("\n=== HỆ THỐNG SẴN SÀNG ===");
  Serial.println("🎯 Chế độ: TỰ ĐỘNG HOÀN TOÀN");
  Serial.println("   - Mở rào: RFID hợp lệ");
  Serial.println("   - Đóng rào: Xe đi qua IR2");
  Serial.println("   - IR1 & IR2: Cập nhật realtime lên Firebase\n");
}

void loop() {
  // Kiểm tra kết nối Firebase
  checkFirebaseConnection();
  
  // Xử lý các cảm biến IR (BỔ SUNG THAM SỐ topicName)
  handleBefore(irVao1, "IR_VAO_1");  // ✅ CẬP NHẬT IR_VAO_1 LÊN FIREBASE
  handleBefore(irRa1, "IR_RA_1");    // ✅ CẬP NHẬT IR_RA_1 LÊN FIREBASE
  
  handleAfter(irVao2, openedByRFID_Vao, "IR_VAO_2", servoVao, servoVaoState, "BARRIER-VAO"); 
  handleAfter(irRa2, openedByRFID_Ra, "IR_RA_2", servoRa, servoRaState, "BARRIER-RA");
  
  // Xử lý parking slots
  handleSlots();
  
  // Xử lý RFID
  handleRFID();
  
  delay(10);
}