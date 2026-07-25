# Surveyor

### **A 2D SLAM Robot — ESP32 · LiDAR-Style Mapping · Autonomous Navigation**

An ESP32-based differential-drive robot that drives via waypoints, performs 360° LiDAR-style obstacle scans with a VL53L0X ToF sensor, cleans and registers the resulting point cloud, and builds a corrected 2D map — all controlled from a phone/browser dashboard served directly off the robot.

No cloud, no companion app, no ROS. One `.ino` file, one WiFi network, one browser tab.




https://github.com/user-attachments/assets/156c11bd-2bc8-4ebe-9245-716b01a7fbd7









---
## Features

- **Manual drive** — on-screen D-pad (hold-to-move) plus fixed-angle turn buttons (90/180/270/360, CW/CCW), with an in-place-rotation jog mode.
- **Waypoint path planning** — tap out a route on a pannable/zoomable grid canvas; the robot turns-and-drives through each point in sequence using encoder + gyro dead reckoning. Supports pause/resume mid-route and live progress on both the dashboard and the onboard OLED.
- **360° obstacle scanning** — the robot rotates itself in place (no pan/tilt servo needed), sampling the ToF sensor every few degrees and converting each reading into a world-frame point.
- **Point cloud cleanup pipeline** — every scan is denoised and simplified into clean wall polylines before being drawn:
  1. **SOR** (Statistical Outlier Removal) — drops points whose neighborhood is statistically sparser than average
  2. **ROR** (Radius Outlier Removal) — drops points with too few neighbors within a fixed radius
  3. **MLS** (Moving Least Squares) — smooths remaining points against their local neighborhood
  4. **DBSCAN** — clusters points into distinct wall segments / objects
  5. Nearest-neighbor ordering — turns each cluster into an ordered polyline
  6. **Chaikin corner-cutting** — smooths the final polyline for a cleaner rendered wall
- **Point-to-Line ICP scan registration** — each completed 360° scan is aligned against the previous one via iterative closest point matching (point-to-line correspondence, outlier-rejected, nearest-neighbor search) to correct for accumulated drift between scans.
- **Lightweight pose graph + loop closure** — every scan is stored as a pose-graph node with a relative-transform edge to its predecessor. When the robot's estimated position comes close to an earlier node, it re-runs ICP against that older scan; if the match is credible, the loop-closure correction is distributed back across the intervening chain (map + poses), pulling the whole route back into consistency.
- **Battery monitoring** — per-cell voltage on a 2S pack via a single junction-tap ADC split, with a configurable low-voltage buzzer alert.
- **Full runtime tuning** — every drive/turn/path/scan constant (including motor-direction inversion flags) is live-editable from the dashboard: Apply (RAM only), Save to EEPROM, or Reset to firmware defaults.
- **Gyro calibration** — rotate-and-mark calibration flow to correct DMP yaw scale, plus a one-tap rebias.
- **Onboard OLED status display** — boot sequence, system self-check, WiFi connect progress, calibration progress, live waypoint/turn/scan status, pause screen, route summary/completion, and fatal-error codes.
- **Connects to your existing WiFi** (station mode, with mDNS) — no separate hotspot to join.

---

## Hardware

| Component | Notes |
|---|---|
| ESP32 dev board | Any variant with enough GPIO; uses WiFi + I2C + PWM + ADC |
| 2x N20 6V gear motors (with built-in encoders) | Quadrature encoder is integrated on the motor's rear shaft, no external encoder disc needed |
| DRV8833 dual H-bridge driver | 2-pin-per-motor PWM drive |
| MPU6050 (IMU) | DMP-based yaw via `MPU6050_6Axis_MotionApps612` — used for heading, turns, and drift correction |
| VL53L0X ToF distance sensor | Drives the 360° scan feature |
| SSD1306 OLED (I2C, 128x64) | Status display, shares the bus with the MPU6050 |
| 2S LiPo/Li-ion pack | Monitored via a resistor-divider junction tap for per-cell voltage |
| Buzzer | Low-battery alert |

### Pin map

| Signal | GPIO |
|---|---|
| Left motor A / B | 27 / 26 |
| Right motor A / B | 25 / 33 |
| Left encoder A / B | 23 / 32 |
| Right encoder A / B | 34 / 35 |
| Battery pack tap (SP) | 36 |
| Battery junction tap (SN) | 39 |
| Buzzer | 5 |
| MPU6050 interrupt (INTA) | 4 |
| I2C (MPU6050, OLED, VL53L0X) | 21 (SDA) / 22 (SCL) |

The VL53L0X's default I2C address (0x29) doesn't collide with the MPU6050 or OLED, so no XSHUT wiring or address remapping is required.

---

## Libraries required

Install via Arduino Library Manager (or as noted):

- `WiFi`, `ESPmDNS`, `WebServer`, `Wire`, `EEPROM` — bundled with the ESP32 core
- `I2Cdev` + `MPU6050_6Axis_MotionApps612` — from [jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib)
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit VL53L0X`

---

## Setup

1. Open the `.ino` file and set your network credentials near the top:
   ```cpp
   const char* WIFI_SSID = "your-network";
   const char* WIFI_PASS = "your-password";
   ```
2. Verify the pin map above matches your actual wiring, and update the battery calibration constants (`PACK_M`/`PACK_C`/`JUNC_M`/`JUNC_C`) for your specific voltage-divider resistors.
3. Flash to the ESP32.
4. Open the Serial Monitor at boot — it prints the assigned IP address once WiFi connects, and the OLED shows the same info plus a QR-free "connected" screen.
5. Browse to `http://<printed-ip>/` or try `http://surveyor.local/` if your device supports mDNS/Bonjour.

---

## Using the dashboard

The dashboard is a single-page, sidebar-navigated dark UI with four panels:

### Manual
D-pad for hold-to-move driving and in-place rotation, plus fixed-angle CW/CCW turn buttons and a STOP button.

### Automatic
- **Path planning** — tap the grid to lay down waypoints (in order), then **Send & Run**. The bot's live position/heading is drawn as a red triangle. Pause/Resume and a "mark here as (0,0)" origin reset are available.
- **360° scan** — start a full-rotation obstacle scan. Raw valid points render live during the scan; once cleanup + ICP registration finish, cleaned wall polylines overlay the same canvas. Scans accumulate on the map until explicitly cleared.

### Calibration
Rotate-by-hand gyro scale calibration (start, rotate to a marked angle, tap the matching Mark button, repeat, Save) plus a one-tap gyro rebias (robot must be stationary).

### Tune
Every speed/turn/path/scan/battery constant and the three motor-direction inversion flags, editable live:
- **Apply** — RAM only, lost on reboot
- **Save to EEPROM** — persists across power cycles
- **Reset to defaults** — reverts to firmware defaults (not saved until you also hit Save)

---

## How the mapping pipeline works

1. **Scan** — the robot rotates in place at a tunable step size, kicking briefly at full speed each step to break static friction, settling, then sampling the ToF sensor.
2. **Registration** — once a full 360° rotation completes, the new scan's points are aligned against the previous scan via point-to-line ICP (nearest-neighbor correspondence with a local-line projection, outlier-rejected, iterated to convergence or a max iteration count).
3. **Pose graph + loop closure** — the scan is recorded as a graph node with an edge to its predecessor. If the robot's current pose is close to an earlier, non-adjacent node, ICP is run against that old scan too; a credible match triggers a loop-closure correction that's linearly distributed across the pose chain between the two nodes, pulling both the map and the live position estimate back into alignment.
4. **Cleanup** — valid points are run through SOR → ROR → MLS → DBSCAN → nearest-neighbor ordering → Chaikin smoothing, producing a set of clean wall polylines separate from the raw point cloud.
5. **Display** — both the raw (registered) point cloud and the cleaned polylines are available from the dashboard and drawn on the same canvas as the path planner.

This is a lightweight, real-time-appropriate approximation of full SLAM (ICP + relaxation-based pose graph) sized for what an ESP32 can actually run — it is not a sparse nonlinear least-squares backend like g2o/Ceres, and is tuned for tens of scan nodes, not hundreds.

---

## Known limitations

- Odometry constants (`leftTicksPerRev`, `rightTicksPerRev`, wheel diameter) should be measured on your actual hardware rather than trusted as shipped defaults — errors here compound into both drive distance and scan registration accuracy.
- Loop closure accepts a match based on a distance/fix-magnitude heuristic, not a full information-matrix uncertainty estimate — in geometrically repetitive environments (long straight corridors, symmetric rooms) it can occasionally match the wrong revisit.
- The pose graph correction is linear-interpolation relaxation across the affected node chain, not a global nonlinear solve — appropriate at small scan counts, but will degrade gracefully rather than optimally on large, densely-looped maps.
- Manual fixed-angle turns and path-following turns use separate tuning constants by design (path-following turns carry more residual momentum, having just finished a drive segment) — if turns feel off, check both.

---

## License

This project is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). See the [LICENSE](./LICENSE) file for details.
