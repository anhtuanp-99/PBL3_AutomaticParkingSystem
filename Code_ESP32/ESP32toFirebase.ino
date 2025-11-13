#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ====== Thư viện tích hợp ======
#include <FirebaseESP32.h>
#include <time.h> // Thư viện thời gian NTP

// ====== WiFi ======
const char* ssid = "Kaka";
const char* password = "12345678";

// ====== Cấu hình Firebase (Lấy từ file test của bạn) ======
#define FIREBASE_HOST "https://car01-351f6-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "snEXxxKZBN2MamzCNPH9B8sp0KTB3wP3nSzh0fvy"

// --- Đối tượng Firebase ---
FirebaseData fbdo; // Đối tượng dùng chung cho các hàm set/push
FirebaseAuth auth;
FirebaseConfig config;

// ====== PIN mapping ======
const int RFID_SCK = 18, RFID_MISO = 19, RFID_MOSI = 23, RFID_SS = 5, RFID_RST = 2;

// Barrier VÀO
const int IR_TRUOC_VAO = 15;
const int IR_SAU_VAO   = 34; // Cảnh báo: Pin 34 là INPUT_ONLY
const int SERVO_VAO_PIN = 21;

// Barrier RA
const int IR_TRUOC_RA = 26;
const int IR_SAU_RA   = 27;
const int SERVO_RA_PIN = 25;

// Parking slots
// Cảnh báo: Pin 32, 33, 34 là INPUT_ONLY
const int IR_PARK[4] = {13, 33, 14, 32};

// Servo góc
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// ====== RFID ======
MFRC522 rfid(RFID_SS, RFID_RST);

// ====== Servo ======
Servo servoVao, servoRa;
bool servoVaoState = false; // false = Đóng, true = Mở
bool servoRaState = false; // false = Đóng, true = Mở

// ====== Thời gian / debounce ======
const unsigned long CONFIRM_MS = 200;       // Debounce cho cảm biến rào cản
const unsigned long SLOT_CONFIRM_MS = 3000; // Debounce cho cảm biến đỗ xe
const unsigned long RFID_DEBOUNCE_MS = 5000; // Thời gian chờ để quét lại cùng 1 thẻ

// ====== Cấu trúc theo dõi IR ======
struct IRTracker {
  int pin;
  int lastRead; // Trạng thái đọc gần nhất
  int stable;   // Trạng thái đã ổn định (đã debounce)
  unsigned long lastChange; // Lần cuối thay đổi trạng thái
  bool sawLowAfterOpen; // Cờ cho logic "xe đã đi qua"
};

IRTracker beforeVao, afterVao, beforeRa, afterRa, slots[4];

bool openedByRFID_Vao = false;
bool openedByRFID_Ra  = false;

// ===========================================
// ====== CÁC HÀM FIREBASE (Từ file test) ======
// ===========================================

// --- Khởi tạo Firebase ---
void initFirebase() {
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);
  Serial.println("✅ Kết nối Firebase thành công!");
}

// --- Hàm lấy ngày hiện tại (yyyy-mm-dd) ---
String getCurrentDate() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("❌ Không lấy được thời gian!");
    return "unknown_date";
  }

  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d", &timeinfo);
  return String(buffer);
}

// --- Hàm lấy giờ hiện tại (hh:mm:ss) ---
String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "00:00:00";
  }

  char buffer[10];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
  return String(buffer);
}

// ==================================================
// ====== CÁC HÀM TRỢ GIÚP FIREBASE (Hàm mới) ======
// ==================================================

/**
 * @brief Ghi một log sự kiện lên Firebase (dùng pushJSON để tạo ID duy nhất)
 * @param type Loại sự kiện (ví dụ: "RFIDScan", "Barrier", "RFIDError")
 * @param message Nội dung thông điệp
 */
void firebaseLog(String type, String message) {
  String date = getCurrentDate();
  if (date == "unknown_date") {
    Serial.println("❌ Không thể log Firebase, chưa đồng bộ thời gian NTP.");
    return;
  }
  String timeNow = getCurrentTime();
  String path = "/logs/" + date; // Ghi log theo ngày

  FirebaseJson json;
  json.set("type", type);
  json.set("message", message);
  json.set("time", timeNow);

  // Chúng ta dùng pushJSON để tạo 1 ID log duy nhất
  if (Firebase.pushJSON(fbdo, path, json)) {
    Serial.printf("✅ FB Log: [%s] %s\n", type.c_str(), message.c_str());
  } else {
    Serial.println("❌ Lỗi log Firebase: " + fbdo.errorReason());
  }
}

/**
 * @brief Cập nhật trạng thái (ghi đè) lên một đường dẫn Firebase
 * @param path Đường dẫn (ví dụ: "parking/slot1", "parking/barrierVao")
 * @param value Giá trị (ví dụ: "CO XE", "OPEN")
 */
void updateFirebaseState(String path, String value) {
  if (Firebase.setString(fbdo, path, value)) {
    Serial.printf("✅ FB State: %s -> %s\n", path.c_str(), value.c_str());
  } else {
    Serial.println("❌ Lỗi cập nhật state Firebase: " + fbdo.errorReason());
  }
}


// ==================================================
// ====== CÁC HÀM LOGIC BÃI ĐỖ XE (Đã sửa) ======
// ==================================================

// ====== Servo điều khiển (Đã sửa để dùng Firebase) ======
void setServo(Servo &s, bool &stateVar, bool open, const char* which, const char* source) {
  // Nếu đã ở đúng trạng thái thì không làm gì
  if (stateVar == open) return;

  int target = open ? OPEN_ANGLE : CLOSED_ANGLE;

  // Ghi trực tiếp, không-chặn (non-blocking)
  s.write(target);
  stateVar = open; // Cập nhật trạng thái

  // Logic reset cờ khi đóng rào cản
  if (strcmp(which, "BARRIER-VAO") == 0 && !open) {
    openedByRFID_Vao = false;
    afterVao.sawLowAfterOpen = false; // Reset luôn cờ này khi đóng
  }
  if (strcmp(which, "BARRIER-RA") == 0 && !open) {
    openedByRFID_Ra = false;
    afterRa.sawLowAfterOpen = false; // Reset luôn cờ này khi đóng
  }

  // --- Thay thế MQTT ---
  String msg = String(which) + (open ? " MỞ (" : " ĐÓNG (") + String(source) + ")";
  // 1. Ghi log sự kiện
  firebaseLog("Barrier", msg); 
  // 2. Cập nhật trạng thái
  String statePath = (strcmp(which, "BARRIER-VAO") == 0) ? "/parking/barrierVao" : "/parking/barrierRa";
  updateFirebaseState(statePath, open ? "OPEN" : "CLOSED");
  // --- Hết thay thế ---

  Serial.printf("[%s] %s bởi %s\n", which, open ? "MỞ" : "ĐÓNG", source);
}

// ====== Khởi tạo IR ======
void initTracker(IRTracker &t, int pin) {
  // CẢNH BÁO QUAN TRỌNG VỀ PHẦN CỨNG:
  if (pin >= 32) {
      pinMode(pin, INPUT);
      Serial.printf("Cảnh báo: Pin %d là INPUT_ONLY. Hãy chắc chắn có điện trở kéo bên ngoài!\n", pin);
  } else {
      pinMode(pin, INPUT_PULLUP);
  }
  
  t.pin = pin;
  t.lastRead = digitalRead(pin);
  t.stable = t.lastRead;
  t.lastChange = millis();
  t.sawLowAfterOpen = false;
}

// ====== Xử lý cảm biến trước (debounce) ======
void handleBefore(IRTracker &t) {
  int r = digitalRead(t.pin);
  if (r != t.lastRead) {
    t.lastChange = millis();
    t.lastRead = r;
  }
  
  // Nếu trạng thái giữ nguyên đủ lâu (CONFIRM_MS), thì coi là ổn định
  if (millis() - t.lastChange >= CONFIRM_MS) {
    t.stable = r;
  }
}

// ====== Xử lý cảm biến sau (logic đóng rào cản) (Đã sửa để dùng Firebase) ======
void handleAfter(IRTracker &tAfter, bool &openedByRFID, const char* topicName, Servo &servo, bool &servoState, const char* which) {
  int r = digitalRead(tAfter.pin);
  
  // Debounce
  if (r != tAfter.lastRead) {
    tAfter.lastChange = millis();
    tAfter.lastRead = r;
  }
  
  // Chỉ xử lý khi trạng thái ổn định thay đổi
  if (millis() - tAfter.lastChange >= CONFIRM_MS && r != tAfter.stable) {
    tAfter.stable = r;
    String topic = String("parking/") + topicName; // Đường dẫn Firebase (ví dụ: parking/IR_SAU_VAO)
    
    if (r == LOW) { // Xe vừa đến cảm biến SAU
      // --- Thay thế MQTT ---
      updateFirebaseState(topic, "CO XE");
      // --- Hết thay thế ---
      
      if (openedByRFID) {
        // Đánh dấu là đã thấy xe đi qua
        tAfter.sawLowAfterOpen = true; 
      }
    } else { // Xe vừa rời cảm biến SAU (r == HIGH)
      // --- Thay thế MQTT ---
      updateFirebaseState(topic, "TRONG");
      // --- Hết thay thế ---
      
      // Chỉ đóng nếu rào cản được mở bằng RFID VÀ chúng ta đã thấy xe đi qua
      if (openedByRFID && tAfter.sawLowAfterOpen) {
        setServo(servo, servoState, false, which, "IR_Passed");
        // Các cờ sẽ được reset bởi hàm setServo khi nó đóng
      }
    }
  }
}

// ====== Parking slot (Đã sửa để dùng Firebase) ======
void handleSlots() {
  for (int i=0;i<4;i++) {
    int r = digitalRead(slots[i].pin);
    if (r != slots[i].lastRead) {
      slots[i].lastChange = millis();
      slots[i].lastRead = r;
    }
    
    // Sử dụng debounce dài hơn cho các vị trí đỗ
    if (millis() - slots[i].lastChange >= SLOT_CONFIRM_MS) {
      if (slots[i].stable != r) {
        slots[i].stable = r;
        String topic = String("parking/slot") + String(i+1);
        String payload = (r==LOW ? "CO XE" : "TRONG"); 
        
        // --- Thay thế MQTT ---
        updateFirebaseState(topic, payload);
        // --- Hết thay thế ---
      }
    }
  }
}

// ====== RFID xử lý (Đã sửa để dùng Firebase) ======
String lastUid = "";
unsigned long lastTrigger = 0;
void handleRFID() {
  // 1. Kiểm tra thẻ mới
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return; // Không có thẻ, thoát
  }

  // 2. Đọc UID
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  // 3. Debounce thẻ
  unsigned long now = millis();
  if (uid == lastUid && (now - lastTrigger < RFID_DEBOUNCE_MS)) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }
  
  lastUid = uid;
  lastTrigger = now;
  
  Serial.printf("RFID Quet: %s\n", uid.c_str());
  // --- Thay thế MQTT ---
  firebaseLog("RFIDScan", uid); // Gửi UID lên Firebase
  // --- Hết thay thế ---

  // 4. Kiểm tra logic hướng
  int vao = beforeVao.stable;
  int ra  = beforeRa.stable;

  if (vao == LOW && ra == HIGH) { // Xe đang ở cổng VÀO
    if (!servoVaoState) { // Nếu rào cản đang đóng
      setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "RFID");
      openedByRFID_Vao = true;
      afterVao.sawLowAfterOpen = false; 
    }
  } else if (ra == LOW && vao == HIGH) { // Xe đang ở cổng RA
    if (!servoRaState) { // Nếu rào cản đang đóng
      setServo(servoRa, servoRaState, true, "BARRIER-RA", "RFID");
      openedByRFID_Ra = true;
      afterRa.sawLowAfterOpen = false; 
    }
  } else {
    // --- Thay thế MQTT ---
    firebaseLog("RFIDError", "HUONG KHONG XAC DINH");
    // --- Hết thay thế ---
    Serial.println("RFID Quet nhung huong khong ro rang (ca hai hoac khong cam bien nao active)");
  }

  // 5. Dọn dẹp
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ====== setup (Đã sửa) ======
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  // Khởi tạo RFID
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
  rfid.PCD_Init();
  Serial.println("RFID Initialized.");

  // Khởi tạo cảm biến
  initTracker(beforeVao, IR_TRUOC_VAO);
  initTracker(afterVao, IR_SAU_VAO);
  initTracker(beforeRa, IR_TRUOC_RA);
  initTracker(afterRa, IR_SAU_RA);
  for (int i=0;i<4;i++) initTracker(slots[i], IR_PARK[i]);
  Serial.println("IR Trackers Initialized.");

  // Khởi tạo Servo
  servoVao.attach(SERVO_VAO_PIN, 500, 2500);
  servoRa.attach(SERVO_RA_PIN, 500, 2500);
  servoVao.write(CLOSED_ANGLE);
  servoRa.write(CLOSED_ANGLE);
  servoVaoState = false; 
  servoRaState = false;
  Serial.println("Servos Initialized.");

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi: ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi đã kết nối!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // --- Tích hợp Firebase & NTP ---
  // Đồng bộ thời gian từ Internet (NTP)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7 (Việt Nam)
  Serial.println("⏰ Đồng bộ thời gian NTP...");

  // Khởi tạo Firebase
  initFirebase();
  // --- Hết phần tích hợp ---

  // (Phần khởi tạo MQTT và callback đã bị xóa)
}

// ====== loop (Đã sửa) ======
void loop() {
  // (Phần kết nối lại và loop MQTT đã bị xóa)

  // 1. Đọc cảm biến
  handleBefore(beforeVao);
  handleBefore(beforeRa);
  handleAfter(afterVao, openedByRFID_Vao, "IR_SAU_VAO", servoVao, servoVaoState, "BARRIER-VAO");
  handleAfter(afterRa, openedByRFID_Ra, "IR_SAU_RA", servoRa, servoRaState, "BARRIER-RA");
  
  handleSlots();

  // 2. Xử lý RFID
  handleRFID();

  // Delay nhỏ để tránh loop() chạy quá nhanh
  delay(5); 
}