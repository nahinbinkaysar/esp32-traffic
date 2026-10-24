#define ENABLE_OLED 1

#include "esp_camera.h"
#if ENABLE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#endif
#include <WebServer.h>
#include <WiFi.h>

// ===== Wi-Fi =====
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ===== OLED =====
// SH1106 I2C OLED (128x64): VCC, GND, SDA, SCL.
// AI Thinker ESP32-CAM has very few spare pins. Use these only if SD card is not used.
#define OLED_SDA 15
#define OLED_SCL 14
#define OLED_ADDR 0x3C
#define OLED_W 128
#define OLED_H 64

#if ENABLE_OLED
Adafruit_SH1106G display(OLED_W, OLED_H, &Wire, -1);
bool oledReady = false;
unsigned long lastOledUpdateAt = 0;
#endif

// ===== AI Thinker ESP32-CAM pins =====
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

// ===== Camera / algorithm settings =====
// QQVGA is 160x120. The ESP32 camera driver usually does not provide a
// native 128x128 mode, so this sketch captures QQVGA and processes a smaller
// ROI/gate region inside it.
const int FRAME_W = 160;
const int FRAME_H = 120;

// Tune this ROI after seeing the web feed.
// For a road video shown on another screen, start with one lane only.
const int ROI_X = 69;
const int ROI_Y = 37;
const int ROI_W = 55;
const int ROI_H = 41;

// Three horizontal gates inside ROI.
// Direction assumed: vehicles move from lower part of image to upper part.
const bool COUNT_BOTTOM_TO_TOP = false;
const int GATE_H = 7;
const int GATE_A_Y = ROI_Y + ROI_H - GATE_H - 2;     // lower gate
const int GATE_B_Y = ROI_Y + (ROI_H - GATE_H) / 2;   // middle gate
const int GATE_C_Y = ROI_Y + 2;                      // upper gate

const int PIXEL_DIFF_THRESHOLD = 24;
const int GATE_CHANGED_PIXELS_THRESHOLD = 55;
const int ROI_CHANGED_PIXELS_THRESHOLD = 180;

const unsigned long STATE_TIMEOUT_MS = 2200;
const unsigned long COOLDOWN_MS = 450;
const unsigned long MAX_COOLDOWN_MS = 1600;
const unsigned long MIN_COUNT_GAP_MS = 900;
const unsigned long REPORT_INTERVAL_MS = 30000; // demo: 30s. use 300000 for 5 minutes.
const int PROCESS_DELAY_MS = 60;                // about 10-15 FPS max capture attempt

WebServer server(80);

uint8_t *latestFrame = nullptr;
uint8_t *backgroundFrame = nullptr;
SemaphoreHandle_t frameMutex;
SemaphoreHandle_t eventMutex;

volatile uint32_t totalCount = 0;
volatile uint32_t intervalCount = 0;
volatile uint32_t lastIntervalCount = 0;
volatile uint32_t processedFrames = 0;
volatile uint32_t gateScoreA = 0;
volatile uint32_t gateScoreB = 0;
volatile uint32_t gateScoreC = 0;
volatile uint32_t roiMotionScore = 0;
volatile uint8_t stateCode = 0;
volatile uint8_t occupancyPercent = 0;
volatile bool gateActiveA = false;
volatile bool gateActiveB = false;
volatile bool gateActiveC = false;
unsigned long intervalStartedAt = 0;
unsigned long lastCountAt = 0;

String eventLog[10];
uint8_t eventLogIndex = 0;

enum CounterState {
  WAIT_FIRST = 0,
  WAIT_SECOND = 1,
  WAIT_THIRD = 2,
  COOLDOWN = 3
};

CounterState counterState = WAIT_FIRST;
unsigned long stateStartedAt = 0;

const char *stateName(uint8_t state) {
  switch (state) {
    case WAIT_FIRST:
      return "WAIT_FIRST_GATE";
    case WAIT_SECOND:
      return "WAIT_SECOND_GATE";
    case WAIT_THIRD:
      return "WAIT_THIRD_GATE";
    case COOLDOWN:
      return "COOLDOWN";
    default:
      return "UNKNOWN";
  }
}

const char *stateShortName(uint8_t state) {
  switch (state) {
    case WAIT_FIRST:
      return "WAIT A";
    case WAIT_SECOND:
      return "WAIT B";
    case WAIT_THIRD:
      return "WAIT C";
    case COOLDOWN:
      return "COOL";
    default:
      return "UNK";
  }
}

#if ENABLE_OLED
void initOled() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);

  oledReady = display.begin(OLED_ADDR, true);
  if (!oledReady) {
    Serial.println("OLED init failed");
    return;
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32 Traffic");
  display.println("OLED ready");
  display.display();
}

void updateOled(bool force = false) {
  if (!oledReady) return;

  unsigned long now = millis();
  if (!force && now - lastOledUpdateAt < 500) return;
  lastOledUpdateAt = now;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Int:");
  display.print((unsigned long)intervalCount);
  display.print("  Tot:");
  display.print((unsigned long)totalCount);

  display.setCursor(0, 12);
  display.print("Last:");
  display.print((unsigned long)lastIntervalCount);
  display.print("  Occ:");
  display.print(occupancyPercent);
  display.print("%");

  display.setCursor(0, 24);
  display.print("Gates ");
  display.print(gateActiveA ? "A" : ".");
  display.print(gateActiveB ? "B" : ".");
  display.print(gateActiveC ? "C" : ".");
  display.print("  ");
  display.print(stateShortName(stateCode));

  display.setCursor(0, 36);
  display.print("A:");
  display.print((unsigned long)gateScoreA);
  display.print(" B:");
  display.print((unsigned long)gateScoreB);

  display.setCursor(0, 48);
  display.print("IP ");
  if (WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP());
  } else {
    display.print("connecting");
  }

  display.display();
}
#endif

void addEvent(const String &message) {
  if (!eventMutex) return;

  if (xSemaphoreTake(eventMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    eventLog[eventLogIndex] = String(millis()) + " ms: " + message;
    eventLogIndex = (eventLogIndex + 1) % 10;
    xSemaphoreGive(eventMutex);
  }
}

bool initCamera() {
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QQVGA);
    s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
  }

  return true;
}

uint32_t motionScoreInRect(const uint8_t *cur, const uint8_t *bg, int x, int y, int w, int h) {
  uint32_t changed = 0;
  for (int yy = y; yy < y + h; yy++) {
    int row = yy * FRAME_W;
    for (int xx = x; xx < x + w; xx++) {
      int idx = row + xx;
      int diff = abs((int)cur[idx] - (int)bg[idx]);
      if (diff > PIXEL_DIFF_THRESHOLD) changed++;
    }
  }
  return changed;
}

void updateBackgroundWhenQuiet(const uint8_t *cur, bool quiet) {
  if (!quiet) return;

  for (int i = 0; i < FRAME_W * FRAME_H; i++) {
    backgroundFrame[i] = (uint8_t)(((uint16_t)backgroundFrame[i] * 15 + cur[i]) / 16);
  }
}

void copyLatestToBackground() {
  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(80)) == pdTRUE) {
    memcpy(backgroundFrame, latestFrame, FRAME_W * FRAME_H);
    xSemaphoreGive(frameMutex);
    addEvent("BACKGROUND RESET");
  }
}

void updateCounterState(bool firstGate, bool secondGate, bool thirdGate, bool anyGate) {
  unsigned long now = millis();
  stateCode = (uint8_t)counterState;

  switch (counterState) {
    case WAIT_FIRST:
      if (firstGate) {
        counterState = WAIT_SECOND;
        stateStartedAt = now;
      }
      break;

    case WAIT_SECOND:
      if (secondGate) {
        counterState = WAIT_THIRD;
        stateStartedAt = now;
      } else if (now - stateStartedAt > STATE_TIMEOUT_MS) {
        counterState = WAIT_FIRST;
      }
      break;

    case WAIT_THIRD:
      if (thirdGate) {
        bool counted = false;
        if (now - lastCountAt >= MIN_COUNT_GAP_MS) {
          totalCount++;
          intervalCount++;
          lastCountAt = now;
          counted = true;
        }
        counterState = COOLDOWN;
        stateStartedAt = now;
        if (counted) {
          Serial.printf("Counted vehicle. interval=%lu total=%lu\n",
                        (unsigned long)intervalCount,
                        (unsigned long)totalCount);
          addEvent("COUNT +1, interval=" + String((unsigned long)intervalCount) +
                   ", total=" + String((unsigned long)totalCount));
        } else {
          addEvent("COUNT SUPPRESSED, min gap");
        }
      } else if (now - stateStartedAt > STATE_TIMEOUT_MS) {
        counterState = WAIT_FIRST;
      }
      break;

    case COOLDOWN:
      if ((!anyGate && now - stateStartedAt > COOLDOWN_MS) ||
          now - stateStartedAt > MAX_COOLDOWN_MS) {
        counterState = WAIT_FIRST;
      }
      break;
  }

  stateCode = (uint8_t)counterState;
}

void processFrame(camera_fb_t *fb) {
  if (!fb || fb->width != FRAME_W || fb->height != FRAME_H) return;

  const uint8_t *cur = fb->buf;

  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    memcpy(latestFrame, cur, FRAME_W * FRAME_H);
    xSemaphoreGive(frameMutex);
  }

  static bool bgReady = false;
  if (!bgReady) {
    memcpy(backgroundFrame, cur, FRAME_W * FRAME_H);
    bgReady = true;
    return;
  }

  uint32_t scoreA = motionScoreInRect(cur, backgroundFrame, ROI_X, GATE_A_Y, ROI_W, GATE_H);
  uint32_t scoreB = motionScoreInRect(cur, backgroundFrame, ROI_X, GATE_B_Y, ROI_W, GATE_H);
  uint32_t scoreC = motionScoreInRect(cur, backgroundFrame, ROI_X, GATE_C_Y, ROI_W, GATE_H);
  uint32_t roiScore = motionScoreInRect(cur, backgroundFrame, ROI_X, ROI_Y, ROI_W, ROI_H);

  gateScoreA = scoreA;
  gateScoreB = scoreB;
  gateScoreC = scoreC;
  roiMotionScore = roiScore;
  occupancyPercent = min(100U, (unsigned int)((roiScore * 100UL) / (ROI_W * ROI_H)));

  bool gateA = scoreA > GATE_CHANGED_PIXELS_THRESHOLD;
  bool gateB = scoreB > GATE_CHANGED_PIXELS_THRESHOLD;
  bool gateC = scoreC > GATE_CHANGED_PIXELS_THRESHOLD;
  gateActiveA = gateA;
  gateActiveB = gateB;
  gateActiveC = gateC;
  bool anyGate = gateA || gateB || gateC;

  if (COUNT_BOTTOM_TO_TOP) {
    updateCounterState(gateA, gateB, gateC, anyGate);
  } else {
    updateCounterState(gateC, gateB, gateA, anyGate);
  }

  bool quiet = roiScore < ROI_CHANGED_PIXELS_THRESHOLD && !anyGate;
  updateBackgroundWhenQuiet(cur, quiet);
  processedFrames++;
}

void processingTask(void *param) {
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      processFrame(fb);
      esp_camera_fb_return(fb);
    }

    unsigned long now = millis();
    if (now - intervalStartedAt >= REPORT_INTERVAL_MS) {
      lastIntervalCount = intervalCount;
      Serial.printf("Interval finished: count=%lu occupancy=%u%% frames=%lu\n",
                    (unsigned long)lastIntervalCount,
                    occupancyPercent,
                    (unsigned long)processedFrames);
      addEvent("INTERVAL DONE, count=" + String((unsigned long)lastIntervalCount) +
               ", occupancy=" + String(occupancyPercent) + "%");
      intervalCount = 0;
      intervalStartedAt = now;
    }

    vTaskDelay(pdMS_TO_TICKS(PROCESS_DELAY_MS));
  }
}

void writeBmpHeader(WiFiClient &client, int width, int height) {
  int rowSize = (width * 3 + 3) & ~3;
  uint32_t imageSize = rowSize * height;
  uint32_t fileSize = 54 + imageSize;
  uint8_t header[54] = {
      'B', 'M',
      0, 0, 0, 0,
      0, 0, 0, 0,
      54, 0, 0, 0,
      40, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      1, 0,
      24, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0x13, 0x0B, 0, 0,
      0x13, 0x0B, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0};

  memcpy(header + 2, &fileSize, 4);
  memcpy(header + 18, &width, 4);
  memcpy(header + 22, &height, 4);
  memcpy(header + 34, &imageSize, 4);
  client.write(header, 54);
}

void drawOverlayPixel(uint8_t *row, int x, uint8_t r, uint8_t g, uint8_t b) {
  int p = x * 3;
  row[p + 0] = b;
  row[p + 1] = g;
  row[p + 2] = r;
}

void handleFrameBmp() {
  uint8_t *copy = (uint8_t *)malloc(FRAME_W * FRAME_H);
  if (!copy) {
    server.send(500, "text/plain", "No memory");
    return;
  }

  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(80)) == pdTRUE) {
    memcpy(copy, latestFrame, FRAME_W * FRAME_H);
    xSemaphoreGive(frameMutex);
  } else {
    free(copy);
    server.send(503, "text/plain", "Frame busy");
    return;
  }

  int rowSize = (FRAME_W * 3 + 3) & ~3;
  uint32_t totalLen = 54 + rowSize * FRAME_H;

  WiFiClient client = server.client();
  server.setContentLength(totalLen);
  server.send(200, "image/bmp", "");
  writeBmpHeader(client, FRAME_W, FRAME_H);

  uint8_t *row = (uint8_t *)calloc(rowSize, 1);
  if (!row) {
    free(copy);
    return;
  }

  for (int yy = FRAME_H - 1; yy >= 0; yy--) {
    memset(row, 0, rowSize);
    for (int xx = 0; xx < FRAME_W; xx++) {
      uint8_t gray = copy[yy * FRAME_W + xx];
      int p = xx * 3;
      row[p + 0] = gray;
      row[p + 1] = gray;
      row[p + 2] = gray;
    }

    bool inRoiTopBottom = yy == ROI_Y || yy == ROI_Y + ROI_H;
    bool inGate = (yy >= GATE_A_Y && yy < GATE_A_Y + GATE_H) ||
                  (yy >= GATE_B_Y && yy < GATE_B_Y + GATE_H) ||
                  (yy >= GATE_C_Y && yy < GATE_C_Y + GATE_H);

    if ((yy >= ROI_Y && yy <= ROI_Y + ROI_H) || inGate) {
      for (int xx = ROI_X; xx < ROI_X + ROI_W && xx < FRAME_W; xx++) {
        if (inRoiTopBottom || xx == ROI_X || xx == ROI_X + ROI_W - 1) {
          drawOverlayPixel(row, xx, 255, 0, 0);
        } else if (inGate) {
          drawOverlayPixel(row, xx, 0, 255, 0);
        }
      }
    }

    client.write(row, rowSize);
  }

  free(row);
  free(copy);
}

void handleStatus() {
  char json[520];
  snprintf(json, sizeof(json),
           "{"
           "\"total_count\":%lu,"
           "\"interval_count\":%lu,"
           "\"last_interval_count\":%lu,"
           "\"occupancy_percent\":%u,"
           "\"gate_a\":%lu,"
           "\"gate_b\":%lu,"
           "\"gate_c\":%lu,"
           "\"gate_a_active\":%s,"
           "\"gate_b_active\":%s,"
           "\"gate_c_active\":%s,"
           "\"roi_motion\":%lu,"
           "\"state\":%u,"
           "\"state_name\":\"%s\","
           "\"processed_frames\":%lu,"
           "\"interval_ms\":%lu"
           "}",
           (unsigned long)totalCount,
           (unsigned long)intervalCount,
           (unsigned long)lastIntervalCount,
           occupancyPercent,
           (unsigned long)gateScoreA,
           (unsigned long)gateScoreB,
           (unsigned long)gateScoreC,
           gateActiveA ? "true" : "false",
           gateActiveB ? "true" : "false",
           gateActiveC ? "true" : "false",
           (unsigned long)roiMotionScore,
           stateCode,
           stateName(stateCode),
           (unsigned long)processedFrames,
           (unsigned long)REPORT_INTERVAL_MS);

  server.send(200, "application/json", json);
}

void handleEvents() {
  String json = "[";
  if (xSemaphoreTake(eventMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (int i = 0; i < 10; i++) {
      int idx = (eventLogIndex + i) % 10;
      if (eventLog[idx].length() == 0) continue;
      if (json.length() > 1) json += ",";
      String escaped = eventLog[idx];
      escaped.replace("\\", "\\\\");
      escaped.replace("\"", "\\\"");
      json += "\"" + escaped + "\"";
    }
    xSemaphoreGive(eventMutex);
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleResetCount() {
  totalCount = 0;
  intervalCount = 0;
  lastIntervalCount = 0;
  lastCountAt = 0;
  counterState = WAIT_FIRST;
  stateCode = WAIT_FIRST;
  addEvent("COUNT RESET");
  server.send(200, "text/plain", "count reset");
}

void handleResetBackground() {
  copyLatestToBackground();
  counterState = WAIT_FIRST;
  stateCode = WAIT_FIRST;
  server.send(200, "text/plain", "background reset");
}

void handleRoot() {
  String html = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-CAM Vehicle Counter</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #eee; }
    main { max-width: 760px; margin: 0 auto; padding: 16px; }
    img { width: 100%; image-rendering: pixelated; border: 1px solid #444; background: #000; }
    .stats { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; margin-top: 12px; }
    .box { background: #1e1e1e; padding: 10px; border-radius: 6px; }
    .label { color: #aaa; font-size: 12px; }
    .value { font-size: 24px; font-weight: 700; }
    .small { font-size: 15px; line-height: 1.35; }
    .on { color: #53ff80; }
    .off { color: #777; }
    button { background: #333; color: #eee; border: 1px solid #555; border-radius: 5px; padding: 9px 12px; margin-right: 8px; }
    button:active { background: #555; }
    ul { padding-left: 20px; color: #ccc; }
    code { color: #9fe870; }
  </style>
</head>
<body>
<main>
  <h2>ESP32-CAM Vehicle Counter</h2>
  <img id="frame" src="/frame.bmp" alt="camera frame">
  <div class="stats">
    <div class="box"><div class="label">Current interval</div><div id="interval" class="value">0</div></div>
    <div class="box"><div class="label">Total</div><div id="total" class="value">0</div></div>
    <div class="box"><div class="label">Occupancy</div><div id="occ" class="value">0%</div></div>
    <div class="box"><div class="label">Gate scores</div><div id="gates" class="value">0 / 0 / 0</div></div>
    <div class="box"><div class="label">Gate active</div><div id="active" class="value small">A / B / C</div></div>
    <div class="box"><div class="label">Counter state</div><div id="state" class="value small">WAIT</div></div>
  </div>
  <p>
    <button onclick="postAction('/reset_count')">Reset count</button>
    <button onclick="postAction('/reset_background')">Reset background</button>
  </p>
  <h3>Event log</h3>
  <ul id="events"></ul>
  <p>Red box is ROI. Green bands are counting gates. Tune constants in the sketch until only the target lane is inside the ROI.</p>
</main>
<script>
async function tick() {
  try {
    const s = await fetch('/status.json').then(r => r.json());
    interval.textContent = s.interval_count;
    total.textContent = s.total_count;
    occ.textContent = s.occupancy_percent + '%';
    gates.textContent = s.gate_a + ' / ' + s.gate_b + ' / ' + s.gate_c;
    active.innerHTML =
      '<span class="' + (s.gate_a_active ? 'on' : 'off') + '">A</span> / ' +
      '<span class="' + (s.gate_b_active ? 'on' : 'off') + '">B</span> / ' +
      '<span class="' + (s.gate_c_active ? 'on' : 'off') + '">C</span>';
    state.textContent = s.state_name;
  } catch (e) {}
}

async function refreshFrame() {
  document.getElementById('frame').src = '/frame.bmp?t=' + Date.now();
}

async function refreshEvents() {
  try {
    const ev = await fetch('/events.json').then(r => r.json());
    events.innerHTML = ev.slice().reverse().map(x => '<li>' + x + '</li>').join('');
  } catch (e) {}
}

async function postAction(path) {
  await fetch(path, { method: 'POST' });
  await tick();
  await refreshEvents();
}

setInterval(tick, 500);
setInterval(refreshFrame, 1600);
setInterval(refreshEvents, 2000);
tick();
refreshFrame();
refreshEvents();
</script>
</body>
</html>
)HTML";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  latestFrame = (uint8_t *)malloc(FRAME_W * FRAME_H);
  backgroundFrame = (uint8_t *)malloc(FRAME_W * FRAME_H);
  frameMutex = xSemaphoreCreateMutex();
  eventMutex = xSemaphoreCreateMutex();

  if (!latestFrame || !backgroundFrame || !frameMutex || !eventMutex) {
    Serial.println("Memory allocation failed");
    while (true) delay(1000);
  }

  memset(latestFrame, 0, FRAME_W * FRAME_H);
  memset(backgroundFrame, 0, FRAME_W * FRAME_H);

#if ENABLE_OLED
  initOled();
#endif

  if (!initCamera()) {
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());

#if ENABLE_OLED
  updateOled(true);
#endif

  server.on("/", handleRoot);
  server.on("/frame.bmp", handleFrameBmp);
  server.on("/status.json", handleStatus);
  server.on("/events.json", handleEvents);
  server.on("/reset_count", HTTP_POST, handleResetCount);
  server.on("/reset_background", HTTP_POST, handleResetBackground);
  server.begin();

  intervalStartedAt = millis();
  xTaskCreatePinnedToCore(processingTask, "processingTask", 8192, nullptr, 1, nullptr, 1);
}

void loop() {
  server.handleClient();
#if ENABLE_OLED
  updateOled();
#endif
  delay(2);
}
