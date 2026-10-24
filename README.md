# ESP32-CAM Vehicle Counter

On-device vehicle counting on an AI Thinker ESP32-CAM. It doesn't use a neural network and doesn't stream to a server: every frame is processed on the microcontroller, which reports counts over Serial, to an OLED and to a small web dashboard.

Built in one day as a proof of concept, and validated on recorded traffic video (see [Validation](#validation)).

## How it works

1. **Capture.** 160x120 grayscale frames (QQVGA), about 10 to 15 per second.
2. **Region of interest.** Only a fixed rectangle covering one lane is analysed.
3. **Background model.** Each pixel is compared with a background frame, and a pixel counts as changed if it differs by more than a threshold. The background is updated as a slow running average, `bg = (15 * bg + frame) / 16`, but **only when the scene is quiet**: no gate is active and overall motion in the region is low. This keeps the reference stable under gradual lighting changes without absorbing a passing vehicle into the background.
4. **Three gates.** Three horizontal bands sit inside the region. A gate is active when enough of its pixels have changed.
5. **Counting state machine.** A vehicle is counted only when the gates activate **in order** (top, then middle, then bottom), so motion that touches a single gate, such as a pedestrian or flicker, isn't counted:
   - `WAIT_FIRST` → `WAIT_SECOND` → `WAIT_THIRD` → count → `COOLDOWN`
   - A partial sequence resets after 2.2 s (`STATE_TIMEOUT_MS`).
   - `COOLDOWN` waits for the gates to clear (minimum 450 ms, maximum 1.6 s), and counts must be at least 900 ms apart (`MIN_COUNT_GAP_MS`). Both prevent one vehicle from being counted twice.
6. **Interval reporting.** Counts are totalled per reporting interval: 30 s for demos, 5 min (`300000` ms) for traffic-count use.

Frame processing runs in its own FreeRTOS task. The web server shares the latest frame and the event log through mutexes, so serving the dashboard doesn't block counting.

## Hardware

- AI Thinker ESP32-CAM with PSRAM
- FTDI programmer for upload
- Stable 5 V supply
- SH1106 I2C OLED, 128x64, address `0x3C`

OLED wiring (assumes the SD card is not used, since it shares these pins):

```text
OLED VCC -> 3.3V
OLED GND -> GND
OLED SDA -> GPIO15
OLED SCL -> GPIO14
```

## Setup

1. Open `esp32_cam_vehicle_counter/esp32_cam_vehicle_counter.ino` in the Arduino IDE.
2. Select board **AI Thinker ESP32-CAM** and enable PSRAM if the menu offers it.
3. Install the **Adafruit SH110X** and **Adafruit GFX** libraries.
4. Set `WIFI_SSID` and `WIFI_PASS`.
5. Upload, open Serial Monitor at `115200`, and visit the printed IP address.

Set `ENABLE_OLED` to `0` to run without the display.

## Dashboard and endpoints

| Endpoint | Purpose |
|---|---|
| `/` | Live debug dashboard: frame with ROI and gates, gate scores, counter state, event log |
| `/frame.bmp` | Latest processed grayscale frame with overlays |
| `/status.json` | Raw algorithm values: gate scores, occupancy, state, counts |
| `/events.json` | Recent counts and interval summaries |
| `POST /reset_count` | Reset counters |
| `POST /reset_background` | Replace the background with the current frame |

The dashboard polls a BMP snapshot about every 1.6 s instead of running a full MJPEG stream, so the camera loop doesn't compete with video streaming.

## Tuning

| Constant | Effect |
|---|---|
| `ROI_X`, `ROI_Y`, `ROI_W`, `ROI_H` | Region covering one lane |
| `PIXEL_DIFF_THRESHOLD` (24) | Brightness change for a pixel to count as changed |
| `GATE_CHANGED_PIXELS_THRESHOLD` (55) | Changed pixels needed to activate a gate |
| `ROI_CHANGED_PIXELS_THRESHOLD` (180) | Above this the scene isn't "quiet", so the background isn't updated |
| `COUNT_BOTTOM_TO_TOP` | Counting direction (default: top to bottom) |

- **Missed vehicles:** lower the thresholds, or move the region to where vehicles look larger and more separated.
- **Double counts:** raise `GATE_CHANGED_PIXELS_THRESHOLD`, lengthen the cooldown, or narrow the region.
- **All gates stuck active:** the region is too large or the thresholds too low. Press *Reset background*.

## Validation

The counter was validated by pointing the ESP32-CAM at recorded traffic video played full-screen on a monitor, then tuning the region and thresholds until the gates fired in sequence for passing vehicles and the event log matched the vehicles crossing the lane.

It has **not** been tested on a real road, and its accuracy has not been measured against manual counts.

## Limitations

- One lane, one direction, with a region and thresholds tuned by hand for a single camera view.
- Sensitive to sudden lighting changes, shadows, reflections and camera shake; the background adapts only slowly and only when the scene is quiet.
- Two vehicles close together, or one occluding another, can be counted as one.
- Testing from a monitor adds screen flicker that a real camera wouldn't see, and misses real-world problems: weather, night, and vibration.

## Next steps

- Measure precision and recall against manually counted ground truth.
- Calibrate thresholds automatically from a quiet period.
- Support multiple lanes and both directions.
- Field-test on a real road with a fixed mount.
