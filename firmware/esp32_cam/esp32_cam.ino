// PillBot wrist camera — ESP32-CAM (AI-Thinker, OV2640). Streams MJPEG over Wi-Fi at a FIXED size
// so LeRobot / OpenCV always get the same frame shape.
//
//   http://<ip>:81/stream     MJPEG stream  (use this URL in configs/robot.yaml -> cameras.wrist.url)
//                             ONE viewer at a time: close browser tabs before PillBot/LeRobot connects
//   http://<ip>/capture       single JPEG
//   http://<ip>/led?v=0..255  flash LED brightness (lights the cap in the jaws; keep it low)
//
// Board: "AI Thinker ESP32-CAM" (esp32 by Espressif). Enable PSRAM if your board menu shows it.
// Flashing: use the ESP32-CAM-MB USB board, or an FTDI adapter (5V, GND, U0R, U0T) with IO0 -> GND
// while resetting; remove IO0-GND and reset again to run. Serial monitor 115200 prints the IP.

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "secrets.h"   // WIFI_SSID_VALUE / WIFI_PASS_VALUE — copy secrets.example.h to secrets.h

// ---- edit these ----
const char* WIFI_SSID = WIFI_SSID_VALUE;   // set in secrets.h
const char* WIFI_PASS = WIFI_PASS_VALUE;
const framesize_t FRAME = FRAMESIZE_QVGA;   // 320x240 — must match width/height in robot.yaml
const int JPEG_QUALITY = 15;               // 10 (better) .. 20 (faster)
const char* HOSTNAME = "pillbot-cam";      // -> http://pillbot-cam.local:81/stream (stable even if the IP changes)
// --------------------

// AI-Thinker pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define LED_GPIO_NUM 4

#define BOUNDARY "pillbotframe"
static const char* STREAM_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char* STREAM_SEP = "\r\n--" BOUNDARY "\r\n";
static const char* PART_HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t ctrl_httpd = NULL, stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t* req) {
  char hdr[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (res == ESP_OK) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t n = snprintf(hdr, sizeof(hdr), PART_HDR, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, hdr, n);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
  return res;
}

static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t led_handler(httpd_req_t* req) {
  char q[32], v[8];
  int duty = 0;
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "v", v, sizeof(v)) == ESP_OK) duty = constrain(atoi(v), 0, 255);
  analogWrite(LED_GPIO_NUM, duty);
  return httpd_resp_sendstr(req, "ok");
}

bool startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t cap = {"/capture", HTTP_GET, capture_handler, NULL};
  httpd_uri_t led = {"/led", HTTP_GET, led_handler, NULL};
  esp_err_t e1 = httpd_start(&ctrl_httpd, &cfg);
  if (e1 == ESP_OK) {
    httpd_register_uri_handler(ctrl_httpd, &cap);
    httpd_register_uri_handler(ctrl_httpd, &led);
  }
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  httpd_uri_t stream = {"/stream", HTTP_GET, stream_handler, NULL};
  esp_err_t e2 = httpd_start(&stream_httpd, &cfg);
  if (e2 == ESP_OK) httpd_register_uri_handler(stream_httpd, &stream);
  Serial.printf("web server :80 %s, stream server :81 %s\n",
                e1 == ESP_OK ? "OK" : esp_err_to_name(e1), e2 == ESP_OK ? "OK" : esp_err_to_name(e2));
  return e1 == ESP_OK && e2 == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAME;
  c.jpeg_quality = JPEG_QUALITY;
  bool psram = psramFound();
  c.fb_count = psram ? 2 : 1;
  c.fb_location = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;           // always the newest frame = lowest latency
  Serial.printf("\nPSRAM: %s\n", psram ? "yes" : "NO (board menu: enable PSRAM / pick AI Thinker ESP32-CAM)");

  // Try 20 MHz, then 10 MHz (some clone boards / sensors only work at 10 MHz)
  const int clocks[] = {20000000, 10000000};
  esp_err_t err = ESP_FAIL;
  for (int clk : clocks) {
    c.xclk_freq_hz = clk;
    err = esp_camera_init(&c);
    Serial.printf("camera init @ %d MHz: %s (0x%x)\n", clk / 1000000, esp_err_to_name(err), err);
    if (err == ESP_OK) break;
    esp_camera_deinit();
    delay(200);
  }
  if (err != ESP_OK) {
    Serial.println("Camera init failed. 0x105/0x20004 = sensor not found: reseat the ribbon "
                   "(contacts facing the board, latch closed). 0x101 = no memory: enable PSRAM.");
    delay(5000);
    ESP.restart();
  }
  sensor_t* sens = esp_camera_sensor_get();
  Serial.printf("sensor PID 0x%x (0x26 = OV2640, 0x3660 = OV3660, 0x5640 = OV5640)\n", sens->id.PID);
  pinMode(LED_GPIO_NUM, OUTPUT);
  analogWrite(LED_GPIO_NUM, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                         // no power-save: steadier stream
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local:81/stream\n", HOSTNAME);
  }
  if (!startServers()) {
    Serial.println("Server start failed — restarting");
    delay(3000);
    ESP.restart();
  }
  Serial.printf("READY pillbot-cam  stream: http://%s:81/stream   photo: http://%s/capture\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 10000) {               // heartbeat: shows it's alive + current IP / signal
    last = millis();
    Serial.printf("alive  ip %s  rssi %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
  if (WiFi.status() != WL_CONNECTED) {         // reconnect if Wi-Fi drops
    Serial.println("Wi-Fi lost, reconnecting");
    WiFi.reconnect();
    delay(2000);
  }
  delay(100);
}
