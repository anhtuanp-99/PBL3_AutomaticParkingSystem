#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// --- CẤU HÌNH CAMERA (CHO BOARD AI THINKER) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
// ---------------------------------------------

// --- THAY ĐỔI CÁC THÔNG SỐ SAU ---
const char* ssid = ":))"; // Tên WiFi của bạn
const char* password = "26262626"; // Mật khẩu WiFi

// ĐÂY LÀ ĐỊA CHỈ IP CỦA MÁY TÍNH CHẠY SERVER PYTHON
const char* serverIp = "192.168.1.92"; //
const int serverPort = 5000;
// ------------------------------------

String serverUrl = "http://" + String(serverIp) + ":" + String(serverPort) + "/upload";

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // --- Khởi động Camera ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 20; // 0-63, lower number means higher quality
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  // -------------------------

  // --- Kết nối WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // ------------------
}

void loop() {
  Serial.println("Taking a photo...");
  camera_fb_t * fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    delay(5000);
    return;
  }

  Serial.printf("Photo taken! Size: %zu bytes\n", fb->len);
  
  // Gửi ảnh lên server
  sendPhoto(fb);

  // Giải phóng bộ nhớ frame buffer
  esp_camera_fb_return(fb);

  Serial.println("Waiting 5 seconds for the next capture...");
  delay(5000); // Chờ 10 giây
}

void sendPhoto(camera_fb_t * fb) {
  HTTPClient http;

  Serial.print("Connecting to server: ");
  Serial.println(serverUrl);
  
  http.begin(serverUrl);
  // Thêm header để server biết đây là dữ liệu ảnh
  http.addHeader("Content-Type", "image/jpeg");

  // Gửi yêu cầu POST với dữ liệu ảnh
  int httpResponseCode = http.POST(fb->buf, fb->len);

  if (httpResponseCode > 0) {
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    String payload = http.getString();
    Serial.println(payload);
  } else {
    Serial.printf("Error code: %d\n", httpResponseCode);
  }

  http.end();
}