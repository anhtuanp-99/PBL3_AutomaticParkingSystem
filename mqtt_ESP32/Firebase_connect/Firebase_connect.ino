#include <WiFi.h>
#include <FirebaseESP32.h>

// --- Cấu hình WiFi ---
#define WIFI_SSID "Kaka"
#define WIFI_PASSWORD "12345678"

// --- Cấu hình Firebase ---
#define FIREBASE_HOST "https://car01-351f6-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "snEXxxKZBN2MamzCNPH9B8sp0KTB3wP3nSzh0fvy"

// --- Khai báo các đối tượng Firebase ---
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

void initFirebase() {
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);
  Serial.println("✅ Kết nối Firebase thành công!");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.print("Đang kết nối WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi đã kết nối!");
  Serial.println(WiFi.localIP());

  initFirebase();
}

void loop() {
  
}
