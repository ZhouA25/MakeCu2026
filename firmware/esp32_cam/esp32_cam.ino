// PillBot wrist camera — ESP32-CAM (AI-Thinker, OV2640). Streams MJPEG over Wi-Fi at a FIXED size
// so LeRobot / OpenCV always get the same frame shape.
//
//   http://<ip>:81/stream     MJPEG stream  (use this URL in configs/robot.yaml -> cameras.wrist.url)
//   http://<ip>/capture       single JPEG
//   http://<ip>/led?v=0..255  flash LED brightness (lights the cap in the jaws; keep it low)
//
// Board: "AI Thinker ESP32-CAM" (esp32 by Espressif). Enable PSRAM if your board menu shows it.
// Flashing: use the ESP32-CAM-MB USB board, or an FTDI adapter (5V, GND, U0R, U0T) with IO0 -> GND
// while resetting; remove IO0-GND and reset again to run. Serial monitor 115200 prints the IP.

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

// ---- edit these ----
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
const framesize_t FRAME = FRAMESIZE_VGA;   // 640x480 — must match width/height in robot.yaml
const int JPEG_QUALITY = 12;               // 10 (better) .. 20 (faster)
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

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t cap = {"/capture", HTTP_GET, capture_handler, NULL};
  httpd_uri_t led = {"/led", HTTP_GET, led_handler, NULL};
  if (httpd_start(&ctrl_httpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(ctrl_httpd, &cap);
    httpd_register_uri_handler(ctrl_httpd, &led);
  }
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  httpd_uri_t stream = {"/stream", HTTP_GET, stream_handler, NULL};
  if (httpd_start(&stream_httpd, &cfg) == ESP_OK) httpd_register_uri_handler(stream_httpd, &stream);
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
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAME;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;           // always the newest frame = lowest latency
  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Camera init failed — check the board selection / ribbon cable");
    delay(3000);
    ESP.restart();
  }
  pinMode(LED_GPIO_NUM, OUTPUT);
  analogWrite(LED_GPIO_NUM, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                         // no power-save: steadier stream
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nREADY pillbot-cam  stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  startServers();
}

void loop() { delay(1000); }
