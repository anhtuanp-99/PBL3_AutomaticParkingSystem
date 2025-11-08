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
const char* mqtt_server = "9e248d5939004b50ba39f2f789574339.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "cloud";
const char* mqtt_password = "Anhtuan123";
WiFiClientSecure espClient;
PubSubClient client(espClient);

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
const unsigned long SLOT_CONFIRM_MS = 3000; // Debounce cho cảm biến đỗ xe (dài hơn để tránh nháy)
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

// ====== MQTT helper ======
void mqttPublish(const char* topic, const char* msg) {
  if (client.connected()) {
    client.publish(topic, msg);
  }
  Serial.printf("[MQTT] %s => %s\n", topic, msg);
}

// ====== Servo điều khiển (ĐÃ SỬA) ======
/*
 * Hàm moveServoSmooth đã bị XÓA.
 * Nó gây ra lỗi "blocking", làm dừng toàn bộ ESP32 trong 800ms.
 * Trong 800ms đó, ESP không thể đọc cảm biến, không thể quét RFID,
 * và có thể bị ngắt kết nối MQTT.
 * Chúng ta sẽ thay thế bằng cách gọi servo.write() trực tiếp.
 */

void setServo(Servo &s, bool &stateVar, bool open, const char* which, const char* source) {
  // Nếu đã ở đúng trạng thái thì không làm gì
  if (stateVar == open) return;

  int target = open ? OPEN_ANGLE : CLOSED_ANGLE;

  // Ghi trực tiếp, không-chặn (non-blocking)
  // Servo sẽ di chuyển nhanh nhất có thể, nhưng hệ thống vẫn phản hồi
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

  String msg = String(which) + (open ? " MỞ (" : " ĐÓNG (") + String(source) + ")";
  mqttPublish("status", msg.c_str());
  Serial.printf("[%s] %s bởi %s\n", which, open ? "MỞ" : "ĐÓNG", source);
}

// ====== Khởi tạo IR ======
void initTracker(IRTracker &t, int pin) {
  t.pin = pin;
  
  // CẢNH BÁO QUAN TRỌNG VỀ PHẦN CỨNG:
  // Các pin 32, 33, 34, 35, 36, 39 là INPUT_ONLY (chỉ đọc)
  // Chúng KHÔNG có điện trở pull-up/pull-down nội.
  // Bạn PHẢI dùng điện trở pull-up (ví dụ 10K Ohm nối 3.3V) bên ngoài
  // cho các pin IR_SAU_VAO (34), IR_PARK[1] (33), IR_PARK[3] (32)
  // nếu cảm biến của bạn là loại collector hở (open-collector).
  if (pin >= 32) {
     pinMode(pin, INPUT);
     Serial.printf("Cảnh báo: Pin %d là INPUT_ONLY. Hãy chắc chắn có điện trở kéo bên ngoài!\n", pin);
  } else {
     // Các pin khác có thể dùng pull-up/pull-down nội.
     // Giả sử cảm biến IR cho mức LOW khi có vật cản,
     // chúng ta dùng INPUT_PULLUP để khi không có vật cản, nó đọc HIGH.
     pinMode(pin, INPUT_PULLUP);
  }
  
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

// ====== Xử lý cảm biến sau (logic đóng rào cản) ======
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
    String topic = String("parking/") + topicName;
    
    if (r == LOW) { // Xe vừa đến cảm biến SAU
      mqttPublish(topic.c_str(), "CO XE");
      if (openedByRFID) {
        // Đánh dấu là đã thấy xe đi qua
        tAfter.sawLowAfterOpen = true; 
      }
    } else { // Xe vừa rời cảm biến SAU (r == HIGH)
      mqttPublish(topic.c_str(), "TRONG");
      // Chỉ đóng nếu rào cản được mở bằng RFID VÀ chúng ta đã thấy xe đi qua
      if (openedByRFID && tAfter.sawLowAfterOpen) {
        setServo(servo, servoState, false, which, "IR_Passed");
        // Các cờ sẽ được reset bởi hàm setServo khi nó đóng
      }
    }
  }
}

// ====== Parking slot (ĐÃ SỬA payload) ======
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
        
        // Tinh chỉnh: Payload chỉ cần trạng thái, topic đã chỉ rõ là slot nào
        String payload = (r==LOW ? "CO XE" : "TRONG"); 
        
        mqttPublish(topic.c_str(), payload.c_str());
      }
    }
  }
}

// ====== RFID xử lý ======
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

  // 3. Debounce thẻ (tránh quét 1 thẻ 10 lần/giây)
  unsigned long now = millis();
  if (uid == lastUid && (now - lastTrigger < RFID_DEBOUNCE_MS)) {
    // Thẻ này vừa được quét, bỏ qua
    rfid.PICC_HaltA(); // Dừng đọc thẻ
    rfid.PCD_StopCrypto1();
    return;
  }
  
  // Nếu là thẻ mới, hoặc đã hết thời gian debounce -> xử lý
  lastUid = uid;
  lastTrigger = now;
  
  Serial.printf("RFID Quet: %s\n", uid.c_str());
  mqttPublish("rfid/scan", uid.c_str()); // Gửi UID lên MQTT

  // 4. Kiểm tra logic hướng
  int vao = beforeVao.stable;
  int ra  = beforeRa.stable;

  if (vao == LOW && ra == HIGH) { // Xe đang ở cổng VÀO
    if (!servoVaoState) { // Nếu rào cản đang đóng
      setServo(servoVao, servoVaoState, true, "BARRIER-VAO", "RFID");
      openedByRFID_Vao = true;
      afterVao.sawLowAfterOpen = false; // Chuẩn bị cờ để theo dõi xe
    }
  } else if (ra == LOW && vao == HIGH) { // Xe đang ở cổng RA
    if (!servoRaState) { // Nếu rào cản đang đóng
      setServo(servoRa, servoRaState, true, "BARRIER-RA", "RFID");
      openedByRFID_Ra = true;
      afterRa.sawLowAfterOpen = false; // Chuẩn bị cờ để theo dõi xe
    }
  } else {
    mqttPublish("status", "HUONG KHONG XAC DINH (RFID)");
    Serial.println("RFID Quet nhung huong khong ro rang (ca hai hoac khong cam bien nao active)");
  }

  // 5. Dọn dẹp
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ====== MQTT callback ======
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char msg[128];
  // Đảm bảo không tràn bộ đệm và luôn kết thúc bằng null
  unsigned int copyLen = min(length, (unsigned int)sizeof(msg)-1);
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';
  Serial.println(msg);

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
  // Loop cho đến khi kết nối lại
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "ESP32_Parking_" + String(random(0xffff), HEX);
    
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      mqttPublish("status", "ESP32 RECONNECTED");
      // Đăng ký topic
      client.subscribe("barrier");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000); // Chờ 2 giây trước khi thử lại
    }
  }
}

// ====== setup ======
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
  servoVaoState = false; // Trạng thái ban đầu là đóng
  servoRaState = false;
  Serial.println("Servos Initialized.");

  // Khởi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // SSL/TLS cho MQTT
  // setInsecure() bỏ qua xác thực chứng chỉ.
  // Tiện lợi cho test, nhưng không an toàn cho production.
  espClient.setInsecure(); 
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  reconnect(); // Kết nối MQTT lần đầu
}

// ====== loop ======
void loop() {
  // 1. Duy trì kết nối MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Rất quan trọng, phải gọi thường xuyên

  // 2. Đọc cảm biến
  handleBefore(beforeVao);
  handleBefore(beforeRa);
  handleAfter(afterVao, openedByRFID_Vao, "IR_SAU_VAO", servoVao, servoVaoState, "BARRIER-VAO");
  handleAfter(afterRa, openedByRFID_Ra, "IR_SAU_RA", servoRa, servoRaState, "BARRIER-RA");
  
  handleSlots();

  // 3. Xử lý RFID (đặt cuối cùng vì nó có logic phức tạp)
  handleRFID();

  // Delay nhỏ để tránh loop() chạy quá nhanh, 
  // nhường thời gian cho các tác vụ nền (như WiFi)
  delay(5); 
}