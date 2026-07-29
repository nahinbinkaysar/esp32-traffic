# ESP32-CAM Vehicle Counter POC

This is a Level 1 proof-of-concept sketch for onboard vehicle counting with an ESP32-CAM.

It does not run YOLO. It uses a lightweight virtual-gate algorithm:

1. Capture `160x120` grayscale frames.
2. Process only a fixed ROI.
3. Compare the ROI against a slow background frame.
4. Measure motion in three horizontal gate bands.
5. Count one vehicle when motion activates the gates in order.
6. Show the latest processed frame and count status in a small web page.

## Hardware

- AI Thinker ESP32-CAM with PSRAM
- FTDI programmer for upload
- Stable 5V supply
- 0.96 inch SSD1306 I2C OLED, usually `128x64`, address `0x3C`

OLED output is enabled in the sketch. Counts are also shown in Serial and the web page.

## OLED Wiring

For a common SSD1306 I2C OLED:

```text
OLED VCC -> ESP32-CAM 3.3V
OLED GND -> ESP32-CAM GND
OLED SDA -> ESP32-CAM GPIO15
OLED SCL -> ESP32-CAM GPIO14
```

Important: `GPIO14` and `GPIO15` are also used by the ESP32-CAM SD card interface. This wiring assumes you are not using the SD card.

If your OLED does not show anything, try changing:

```cpp
#define OLED_ADDR 0x3C
```

to:

```cpp
#define OLED_ADDR 0x3D
```

## Setup

1. Open `esp32_cam_vehicle_counter.ino` in Arduino IDE.
2. Install/select ESP32 board support.
3. Board: `AI Thinker ESP32-CAM`.
4. Enable PSRAM if your board menu exposes it.
5. Install Arduino libraries:

```text
Adafruit SSD1306
Adafruit GFX Library
```

6. Change:

```cpp
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
```

7. Upload.
8. Open Serial Monitor at `115200`.
9. Visit the printed IP address in a browser.

To temporarily disable OLED support:

```cpp
#define ENABLE_OLED 0
```

## Demo Method

For your current prototype:

1. Play the traffic video full-screen on a laptop/monitor.
2. Mount the ESP32-CAM so the image does not shake.
3. Open the ESP32 web page.
4. Tune the ROI constants until the red box covers one target lane/strip only.
5. Tune thresholds until the green gates activate when a vehicle passes.

Start with a 30-second report interval:

```cpp
const unsigned long REPORT_INTERVAL_MS = 30000;
```

For the real 5-minute interval:

```cpp
const unsigned long REPORT_INTERVAL_MS = 300000;
```

## Tuning Constants

Main constants to tune:

```cpp
const int ROI_X = 69;
const int ROI_Y = 37;
const int ROI_W = 55;
const int ROI_H = 41;

const int PIXEL_DIFF_THRESHOLD = 24;
const int GATE_CHANGED_PIXELS_THRESHOLD = 55;
const int ROI_CHANGED_PIXELS_THRESHOLD = 180;
```

If it misses vehicles:

- Lower `PIXEL_DIFF_THRESHOLD`.
- Lower `GATE_CHANGED_PIXELS_THRESHOLD`.
- Move the ROI to where vehicles appear larger and more separated.

If it double-counts:

- Increase `GATE_CHANGED_PIXELS_THRESHOLD`.
- Increase `COOLDOWN_MS`.
- Make the ROI narrower.

If screen flicker causes false motion:

- Increase `PIXEL_DIFF_THRESHOLD`.
- Lower monitor brightness a little.
- Keep the ESP32-CAM exposure stable by avoiding reflections.

## Web Feed Performance

The sketch does not use the standard MJPEG camera stream. The camera loop processes frames continuously and stores the latest grayscale frame in RAM. The web page asks for that latest frame as a BMP snapshot every 1600 ms, while status updates refresh faster.

This still costs Wi-Fi and CPU time, but it is much cheaper than competing with the camera for full JPEG streaming. For best counting performance during a demo, keep only one browser tab open.

If the feed lags badly:

- Keep only one browser tab open.
- Move the ESP32-CAM closer to the Wi-Fi router or hotspot.
- Increase the `refreshFrame` interval in the dashboard JavaScript.
- Watch `/status.json` instead of the image feed while tuning thresholds.

## Verification Outputs

Use these during tuning and supervisor demo:

- `/` shows the live debug dashboard.
- `/frame.bmp` shows the latest processed grayscale frame with ROI and gates overlaid.
- `/status.json` shows raw algorithm values.
- `/events.json` shows recent count and interval events.
- Serial Monitor at `115200` prints every counted vehicle and every interval summary.

On the dashboard:

- `Gate scores` should rise when motion crosses the green bands.
- `Gate active` should light up in order: `A -> B -> C` for bottom-to-top counting.
- `Counter state` should move through `WAIT_FIRST_GATE`, `WAIT_SECOND_GATE`, `WAIT_THIRD_GATE`, then `COOLDOWN`.
- `Event log` should add `COUNT +1` only after the full gate sequence is observed.

If all three gates stay active constantly, the ROI is too large or thresholds are too low.
If no gate becomes active, lower the thresholds or move the ROI closer to larger vehicles.

If the count stays at `1`, check `Counter state`.

- If it stays on `COOLDOWN`, the gates are not going inactive. Increase `GATE_CHANGED_PIXELS_THRESHOLD`, shrink the ROI, or press `Reset background`.
- If it stays on `WAIT_SECOND_GATE`, Gate A is active but Gate B is not. Move the ROI/gates so vehicle motion crosses A then B.
- If it stays on `WAIT_THIRD_GATE`, Gate C is not being activated. Move the ROI upward/downward or lower the threshold.
