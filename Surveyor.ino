/*
  MappingBot - Straight-line driving + precision turns using MPU6050 DMP yaw.
  PATH PLANNING: a 20cm-grid canvas on the dashboard where you tap out a
  route, the bot is drawn as a red triangle (live position + heading), and
  "Send & Run" makes the bot turn-and-drive through each waypoint in turn
  using dead-reckoning (encoders + DMP yaw).

  360 OBSTACLE SCAN: the bot rotates itself in place (no separate servo),
  pausing every scanStepDeg degrees to take a VL53L0X range reading, and
  converts each reading to a world-frame (x,y) point using the bot's
  position/heading at that instant. Results are overlaid as amber dots on
  the same path-planning canvas.

  DASHBOARD UI: sidebar-navigated dark-theme dashboard with four panels:
    - Manual    : D-pad drive + fixed-angle turn buttons + STOP
    - Automatic : the waypoint canvas / path planner + 360 scan controls
    - Calibration: gyro turn-scale calibration + rebias
    - Tune      : every drive/turn/path/scan tuning parameter (including the
                  motor-direction booleans) is live-editable from the
                  browser, with Apply (RAM only), Save to EEPROM, and
                  Reset to firmware defaults.

  Connects to YOUR WiFi network (station mode) instead of creating its own
  hotspot. After it connects, open the Serial Monitor to see the IP address
  it was given, then type that IP into a browser to open the dashboard.
  (It also advertises itself as http://mappingbot.local if your device
  supports mDNS/Bonjour, so you can try that instead of the raw IP.)
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

// Must be defined here, near the top - Arduino IDE auto-generates forward
// declarations for every function and inserts them right after the
// includes. Since several functions below return/accept these structs,
// they have to exist before that point or the auto-generated prototypes
// fail to compile.
struct Transform2D { float dx, dy, dTheta; };
struct CleanPoint { float x, y; };
#define MAX_POLYLINE_POINTS_FWD 100  // must match MAX_POLYLINE_POINTS below - see note
struct Polyline { CleanPoint pts[MAX_POLYLINE_POINTS_FWD]; int count; };

// =========================================================================
// WIFI SETTINGS  -- put your home/hotspot network's name and password here
// =========================================================================
const char* WIFI_SSID = "your-network";      // <-- change this
const char* WIFI_PASS = "your-password";  // <-- change this
const char* HOSTNAME  = "mappingbot";          // dashboard at http://mappingbot.local

// =========================================================================
// HARDWARE PINS
// =========================================================================

// Motor driver inputs (2 pins per motor, PWM drives one side high)
#define AIN1 27   // Left motor  - forward/reverse pin A
#define AIN2 26   // Left motor  - forward/reverse pin B
#define BIN1 25   // Right motor - forward/reverse pin A
#define BIN2 33   // Right motor - forward/reverse pin B

// Wheel encoders (only used to show distance travelled on the dashboard)
#define LEFT_C1 23
#define LEFT_C2 32
#define RIGHT_C1 34
#define RIGHT_C2 35

// Battery monitoring (2S pack, junction-tap split for per-cell voltage)
#define PIN_PACK     36   // SP
#define PIN_JUNCTION 39   // SN

// Low-voltage buzzer
#define BUZZER_PIN 5

// MPU6050 interrupt pin - wire the sensor's INTA output here.
// Lets the ESP32 read new gyro data the instant it's ready instead of
// constantly polling the sensor over I2C.
#define MPU_INT_PIN 4

// VL53L0X ToF sensor shares the I2C bus (Wire, pins 21/22) with the MPU6050
// and OLED. Default address 0x29 doesn't collide with either, so no XSHUT
// wiring or address remapping is needed.

// =========================================================================
// TUNING PARAMETERS - firmware defaults
// =========================================================================
// Everything in this section used to be a hard #define. It's now a live,
// runtime-editable global (see "TUNABLE GLOBALS" below) that can be changed
// from the dashboard's Tune panel, applied instantly, saved to EEPROM, or
// reset back to these DEFAULT_* values at any time. The #defines that
// remain below are structural (pins, PWM channel setup, grid size that the
// browser canvas math assumes) rather than "tuning" in the driving sense.

// --- Motor PWM (structural - changing these needs a re-attach, not exposed to Tune) ---
#define PWM_FREQ 5000   // PWM switching frequency (Hz) - shouldn't need to change
#define PWM_RES  8      // PWM resolution in bits -> 0-255 duty cycle range

// --- Motor PWM / speed defaults ---
const int   DEFAULT_BASE_SPEED        = 120;   // Straight-line driving speed (0-255)
const float DEFAULT_LEFT_SPEED_SCALE  = 1.07;  // Per-wheel speed trim
const float DEFAULT_RIGHT_SPEED_SCALE = 1.0;

// --- Distance calculation (encoder ticks -> mm) defaults ---
const float DEFAULT_LEFT_TICKS_PER_REV  = 283.0;
const float DEFAULT_RIGHT_TICKS_PER_REV = 257.0;
const float DEFAULT_WHEEL_DIAMETER_MM   = 44.0;

// --- Drive-straight correction default ---
const float DEFAULT_HEADING_KP = 5.0;

// --- Turning defaults ---
const int   DEFAULT_TURN_SPEED_MAX     = 120;
const int   DEFAULT_TURN_SPEED_MIN     = 80;
const float DEFAULT_TURN_SLOWDOWN_DEG  = 30;
const float DEFAULT_TURN_TOLERANCE_DEG = 0.3;
const float DEFAULT_TURN_STOP_LEAD_DEG = 6;

// --- Path following defaults ---
const float DEFAULT_PATH_ARRIVE_TOL_MM = 5;
const int8_t DEFAULT_PATH_HEADING_SIGN = -1;  // flip to +1 if paths turn the wrong way

// --- Motor direction inversion defaults ---
const bool DEFAULT_INVERT_LEFT  = true;
const bool DEFAULT_INVERT_RIGHT = true;
const bool DEFAULT_INVERT_TURN  = false;

// --- Battery protection default ---
const float DEFAULT_MIN_CELL_VOLTAGE = 3.60;  // volts; either cell below this -> buzzer alert

// --- 360 scan defaults ---
const float DEFAULT_SCAN_STEP_DEG   = 6.0;   // degrees rotated between each ToF sample (60 points per rotation)
const int   DEFAULT_SCAN_SETTLE_MS  = 40;    // pause after stopping rotation, before reading ToF (let vibration die down)
const int   DEFAULT_SCAN_KICK_MS    = 60;    // brief full-speed pulse to break static friction each scan step

// =========================================================================
// TUNABLE GLOBALS - live values, loaded from EEPROM at boot (or defaults
// above if nothing has been saved yet). Every value in this block can be
// changed live from the dashboard's Tune panel.
// =========================================================================
int   baseSpeed        = DEFAULT_BASE_SPEED;
float leftSpeedScale   = DEFAULT_LEFT_SPEED_SCALE;
float rightSpeedScale  = DEFAULT_RIGHT_SPEED_SCALE;

float leftTicksPerRev  = DEFAULT_LEFT_TICKS_PER_REV;
float rightTicksPerRev = DEFAULT_RIGHT_TICKS_PER_REV;
float wheelDiameterMM  = DEFAULT_WHEEL_DIAMETER_MM;

float headingKP = DEFAULT_HEADING_KP;

int   turnSpeedMax     = DEFAULT_TURN_SPEED_MAX;
int   turnSpeedMin     = DEFAULT_TURN_SPEED_MIN;
float turnSlowdownDeg  = DEFAULT_TURN_SLOWDOWN_DEG;
float turnToleranceDeg = DEFAULT_TURN_TOLERANCE_DEG;
float turnStopLeadDeg  = DEFAULT_TURN_STOP_LEAD_DEG;

float pathArriveTolMM  = DEFAULT_PATH_ARRIVE_TOL_MM;
int8_t pathHeadingSign = DEFAULT_PATH_HEADING_SIGN;

bool invertLeft  = DEFAULT_INVERT_LEFT;
bool invertRight = DEFAULT_INVERT_RIGHT;
bool invertTurn  = DEFAULT_INVERT_TURN;

float minCellVoltage = DEFAULT_MIN_CELL_VOLTAGE;

float scanStepDeg  = DEFAULT_SCAN_STEP_DEG;
int   scanSettleMs = DEFAULT_SCAN_SETTLE_MS;
int   scanKickMs   = DEFAULT_SCAN_KICK_MS;

// Wheel circumference depends on the tunable wheelDiameterMM, so it's now a
// small helper function instead of a compile-time #define.
float wheelCircumferenceMM() {
  return PI * wheelDiameterMM;
}

// =========================================================================
// PATH PLANNING (structural - tied to the browser canvas grid, not tuned)
// =========================================================================
#define GRID_CELL_MM     100.0  // One grid square on the dashboard = 10cm
#define MAX_PATH_POINTS   30     // Max waypoints per uploaded path

// =========================================================================
// 360 SCAN (structural)
// =========================================================================
#define MAX_SCAN_POINTS   400    // shared across multiple accumulated scans; raise further if you scan often before clearing

// =========================================================================
// OLED DISPLAY (SSD1306, I2C, shares the bus with the MPU6050)
// =========================================================================
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C
#define OLED_UPDATE_MS 200   // how often the runtime status screen refreshes

// ---- Battery calibration (from your Battery_Read.ino) ----
const float PACK_M = 0.00299415;
const float PACK_C = 0.415365;
const float JUNC_M = 0.00345697;
const float JUNC_C = -0.344673;
const int BATTERY_NUM_SAMPLES = 16;
const unsigned long BATTERY_UPDATE_MS = 2000; // read battery every 2s

// =========================================================================
// CALIBRATION / TUNING EEPROM LAYOUT
// =========================================================================
#define EEPROM_SIZE 128
#define CAL_MAGIC 0xC0FFEE03    // Changing the CalibData layout? Bump this so old EEPROM data is ignored.
#define TUNE_MAGIC 0xC0FFEE12   // bumped: added scanStepDeg/scanSettleMs
#define TUNE_EEPROM_ADDR 32     // CalibData lives at address 0 (8 bytes); Tune starts well clear of it.

// What gets saved to EEPROM by the "Save to EEPROM" button in Calibration
struct CalibData {
  uint32_t magic;   // used to check the EEPROM actually holds valid data
  float scale;      // multiplier applied on top of raw DMP yaw to correct turn accuracy
};

// What gets saved to EEPROM by the "Save to EEPROM" button in Tune
struct TuneData {
  uint32_t magic;
  int32_t baseSpeed;
  float   leftSpeedScale;
  float   rightSpeedScale;
  float   leftTicksPerRev;
  float   rightTicksPerRev;
  float   wheelDiameterMM;
  float   headingKP;
  int32_t turnSpeedMax;
  int32_t turnSpeedMin;
  float   turnSlowdownDeg;
  float   turnToleranceDeg;
  float   turnStopLeadDeg;
  float   pathArriveTolMM;
  int8_t  pathHeadingSign;
  uint8_t invertLeft;
  uint8_t invertRight;
  uint8_t invertTurn;
  float   minCellVoltage;
  float   scanStepDeg;
  int32_t scanSettleMs;
};

// =========================================================================
// GLOBAL OBJECTS & STATE
// =========================================================================
WebServer server(80);

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledOk = false;   // set true in setup() if the OLED responds
unsigned long lastOledUpdateMillis = 0;
float cell1Voltage = 0.0f;
float cell2Voltage = 0.0f;
unsigned long lastBatteryUpdateMillis = 0;

bool lowBatteryAlert = false;
bool buzzerOn = false;
unsigned long buzzerLastToggleMillis = 0;
const unsigned long BUZZER_BEEP_MS = 200;   // beep on-time
const unsigned long BUZZER_GAP_MS  = 800;   // silence between beeps

// --- Fatal error state (shown on OLED, halts further operation) ---
bool errorActive = false;
String errorMessage = "";
String errorCode = "";

// --- Path pause/resume ---
bool pathPaused = false;

// --- Path timing/stats, used for the ROUTE SUMMARY and ROUTE COMPLETE screens ---
unsigned long pathStartMillis = 0;
float pathTotalLengthMM = 0;         // computed once when a path is uploaded

// Route-complete screen latches for a few seconds after the last waypoint,
// since pathFollowing itself flips back to false immediately on completion.
bool showRouteComplete = false;
unsigned long routeCompleteShownAtMillis = 0;
const unsigned long ROUTE_COMPLETE_DISPLAY_MS = 4000;
unsigned long routeCompleteElapsedMs = 0;
int routeCompletePoints = 0;

MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
uint16_t packetSize = 0;
Quaternion q;
VectorFloat gravity;
float ypr[3];

Adafruit_VL53L0X tof = Adafruit_VL53L0X();
bool tofOk = false;   // set true in setup() if the sensor responds

// Wheel encoder tick counters, updated in the ISRs below
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// High-level robot state - only one of these should be true at a time
bool moving = false;        // manual drive active (dashboard D-pad button held)
bool turning = false;       // mid manual fixed-angle turn (Turns panel)
bool calibrating = false;   // gyro calibration session active
bool pathFollowing = false; // executing an uploaded path
bool scanning = false;      // executing a 360 obstacle scan

// Which D-pad button is currently held, while moving == true.
enum ManualDir { DIR_NONE, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT };
ManualDir manualDir = DIR_NONE;

// The dashboard must "ping" every ~150ms while the move button is held;
// if pings stop (phone locks, wifi drops, etc.) the bot auto-stops.
unsigned long lastPingMillis = 0;
const unsigned long PING_TIMEOUT_MS = 500;

// Continuous (unwrapped) heading in degrees, accumulated from DMP yaw deltas.
// Reset to 0 at the start of each forward drive / turn / path segment / scan -
// used only as a LOCAL progress tracker for whichever motion is currently running.
float headingDeg = 0;

// Tracks the last raw DMP yaw reading so we can compute the wrapped delta
// frame-to-frame (raw DMP yaw wraps at +-180 degrees).
float lastYawDeg = 0;
bool haveLastYaw = false;

// --- Turn state (manual turns) ---
float turnStartHeading = 0;  // headingDeg captured when the turn began
float turnTargetDelta = 0;   // how many degrees this turn needs to cover
int   turnDirSign = 1;       // +1 or -1 depending on cw/ccw requested

// --- Calibration scale factor ---
// Loaded from EEPROM at boot. 1.0 = no correction. This is what the
// calibration routine tunes and what "Save to EEPROM" writes out.
float scale = 1.0;

// --- Calibration session state (only meaningful while calibrating == true) ---
float calHeading = 0;    // raw (unscaled) heading accumulated since /calib/start
float scaleSum = 0;      // running total of scale samples from each "Mark" press
int   scaleCount = 0;    // how many "Mark" samples have been taken this session
float lastMarkScale = 0; // scale computed from the most recent "Mark" press

// --- Live world position (dead reckoning) ---
// (0,0) = wherever the bot was when it booted (or last /path/reset).
// heading 0 = "up"/"north" on the dashboard grid; positive heading rotates
// toward +X (right), matching the canvas's rotate() direction.
float botX = 0, botY = 0;      // mm
float globalHeading = 0;       // degrees, unbounded (not wrapped) - fine for atan2 math which wraps itself
float lastAvgDistMM = 0;       // avgDist reading at the last odometry update, for computing deltas

// --- Path following state ---
struct PathPoint { float x, y; };  // mm, world frame
PathPoint pathPoints[MAX_PATH_POINTS];
int pathLength = 0;
int pathIndex = 0;
enum PathSubState { PATH_IDLE, PATH_TURNING, PATH_DRIVING };
PathSubState pathSubState = PATH_IDLE;
float pathSegTargetDist = 0;  // mm to travel for the current DRIVE sub-segment

// --- 360 scan state ---
struct ScanPoint { float x, y; bool valid; }; // world mm; valid=false means "no return / out of range"
ScanPoint scanPoints[MAX_SCAN_POINTS];
int scanPointCount = 0;

float scanStartHeading = 0;   // headingDeg captured when the scan began
float scanTraveled = 0;       // degrees rotated so far this scan
enum ScanSubState { SCAN_ROTATING, SCAN_SETTLING, SCAN_SAMPLING };
ScanSubState scanSubState = SCAN_ROTATING;
unsigned long scanSettleStartMillis = 0;
bool scanKicking = false;
unsigned long scanKickStartMillis = 0;
int scanPointsThisRun = 0;   // points taken since THIS scan started, resets each /scan/start (separate from the accumulated scanPointCount total)

// =========================================================================
// SCAN REGISTRATION (Point-to-Line ICP) + LIGHTWEIGHT POSE GRAPH
// =========================================================================
#define MAX_POSE_NODES 20
#define ICP_MAX_ITERATIONS 8
#define ICP_MAX_CORRESPONDENCE_MM 150.0   // tightened from 250 - stops matching points against the wrong, distant wall
#define ICP_CONVERGE_MM 2.0
#define LOOP_CLOSURE_DIST_MM 300.0
#define LOOP_CLOSURE_MIN_NODE_GAP 3
#define ICP_MIN_INLIERS 8                 // reject a fit if fewer than this many points actually matched
#define ICP_MAX_CORRECTION_MM 150.0       // reject a fit whose translation correction is implausibly large
#define ICP_MAX_CORRECTION_DEG 25.0       // reject a fit whose rotation correction is implausibly large

struct PoseNode {
  float x, y, headingDeg;
  int scanStartIdx, scanCount;
};
PoseNode poseNodes[MAX_POSE_NODES];
int poseNodeCount = 0;

struct PoseEdge {
  int fromNode, toNode;
  float dx, dy, dHeadingDeg;
};
PoseEdge poseEdges[MAX_POSE_NODES * 2];
int poseEdgeCount = 0;

int scanRunStartIdx = 0;   // set in handleScanStart(), read in finalizeScanNode()
int icpLastInlierCount = 0; // how many point-pairs the last ICP call actually matched - low count = untrustworthy fit

// Closed-form 2D rigid-transform fit (Horn's method, 2D case) between two
// matched point sets.
Transform2D solveRigidTransform(float* ax, float* ay, float* bx, float* by, int n) {
  Transform2D t = {0, 0, 0};
  if (n < 2) return t;

  float axMean = 0, ayMean = 0, bxMean = 0, byMean = 0;
  for (int i = 0; i < n; i++) { axMean += ax[i]; ayMean += ay[i]; bxMean += bx[i]; byMean += by[i]; }
  axMean /= n; ayMean /= n; bxMean /= n; byMean /= n;

  float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
  for (int i = 0; i < n; i++) {
    float axc = ax[i] - axMean, ayc = ay[i] - ayMean;
    float bxc = bx[i] - bxMean, byc = by[i] - byMean;
    Sxx += axc * bxc; Sxy += axc * byc;
    Syx += ayc * bxc; Syy += ayc * byc;
  }

  float theta = atan2(Sxy - Syx, Sxx + Syy);
  float cosT = cos(theta), sinT = sin(theta);

  t.dTheta = theta * 180.0 / PI;
  t.dx = bxMean - (cosT * axMean - sinT * ayMean);
  t.dy = byMean - (sinT * axMean + cosT * ayMean);
  return t;
}

// Point-to-line ICP: for each source point, finds the nearest reference
// point AND that point's own nearest neighbor, forms a short local line
// segment between them, and projects the source point onto that line -
// this is what makes it point-to-LINE (converges better on flat walls)
// rather than point-to-point.
Transform2D icpAlignToReference(float* srcX, float* srcY, int srcCount,
                                 float* refX, float* refY, int refCount,
                                 float initHeadingDeg) {
  Transform2D total = {0, 0, initHeadingDeg};
  if (srcCount < 3 || refCount < 3) return total;

  static float wx[80], wy[80];
  int n = min(srcCount, 80);
  for (int i = 0; i < n; i++) { wx[i] = srcX[i]; wy[i] = srcY[i]; }

  float corrAx[80], corrAy[80], corrBx[80], corrBy[80];

  for (int iter = 0; iter < ICP_MAX_ITERATIONS; iter++) {
    int m = 0;
    float meanCorrection = 0;

    for (int i = 0; i < n; i++) {
      int nearestIdx = -1;
      float nearestDist = 1e9;
      for (int j = 0; j < refCount; j++) {
        float ddx = wx[i] - refX[j], ddy = wy[i] - refY[j];
        float d = sqrt(ddx * ddx + ddy * ddy);
        if (d < nearestDist) { nearestDist = d; nearestIdx = j; }
      }
      if (nearestIdx < 0 || nearestDist > ICP_MAX_CORRESPONDENCE_MM) continue;

      int neighborIdx = -1;
      float neighborDist = 1e9;
      for (int j = 0; j < refCount; j++) {
        if (j == nearestIdx) continue;
        float ddx = refX[nearestIdx] - refX[j], ddy = refY[nearestIdx] - refY[j];
        float d = sqrt(ddx * ddx + ddy * ddy);
        if (d < neighborDist) { neighborDist = d; neighborIdx = j; }
      }

      float targetX = refX[nearestIdx], targetY = refY[nearestIdx];
      if (neighborIdx >= 0 && neighborDist < ICP_MAX_CORRESPONDENCE_MM) {
        float lx = refX[neighborIdx] - refX[nearestIdx];
        float ly = refY[neighborIdx] - refY[nearestIdx];
        float lenSq = lx * lx + ly * ly;
        if (lenSq > 1.0) {
          float t = ((wx[i] - refX[nearestIdx]) * lx + (wy[i] - refY[nearestIdx]) * ly) / lenSq;
          t = constrain(t, 0.0f, 1.0f);
          targetX = refX[nearestIdx] + t * lx;
          targetY = refY[nearestIdx] + t * ly;
        }
      }

      corrAx[m] = wx[i]; corrAy[m] = wy[i];
      corrBx[m] = targetX; corrBy[m] = targetY;
      meanCorrection += sqrt(sq(targetX - wx[i]) + sq(targetY - wy[i]));
      m++;
    }

    if (m < 3) break;
    meanCorrection /= m;
    icpLastInlierCount = m;

    Transform2D step = solveRigidTransform(corrAx, corrAy, corrBx, corrBy, m);

    float cosT = cos(step.dTheta * PI / 180.0), sinT = sin(step.dTheta * PI / 180.0);
    for (int i = 0; i < n; i++) {
      float rx = cosT * wx[i] - sinT * wy[i] + step.dx;
      float ry = sinT * wx[i] + cosT * wy[i] + step.dy;
      wx[i] = rx; wy[i] = ry;
    }

    total.dx += step.dx;
    total.dy += step.dy;
    total.dTheta += step.dTheta;

    if (meanCorrection < ICP_CONVERGE_MM) break;
  }

  return total;
}

// Distributes a loop-closure correction across the node chain by linear
// interpolation - a lightweight relaxation, not full nonlinear
// optimization, but genuinely appropriate at tens-of-nodes scale.
void distributeLoopCorrection(int fromNode, int toNode, Transform2D loopFix) {
  int chainLen = toNode - fromNode;
  if (chainLen <= 0) return;

  for (int k = fromNode + 1; k <= toNode; k++) {
    float frac = (float)(k - fromNode) / (float)chainLen;
    float corrX = loopFix.dx * frac;
    float corrY = loopFix.dy * frac;
    float corrTheta = loopFix.dTheta * frac;

    poseNodes[k].x += corrX;
    poseNodes[k].y += corrY;
    poseNodes[k].headingDeg += corrTheta;

    float cosT = cos(corrTheta * PI / 180.0), sinT = sin(corrTheta * PI / 180.0);
    for (int p = poseNodes[k].scanStartIdx; p < poseNodes[k].scanStartIdx + poseNodes[k].scanCount; p++) {
      float rx = cosT * scanPoints[p].x - sinT * scanPoints[p].y + corrX;
      float ry = sinT * scanPoints[p].x + cosT * scanPoints[p].y + corrY;
      scanPoints[p].x = rx;
      scanPoints[p].y = ry;
    }
  }

  botX = poseNodes[toNode].x;
  botY = poseNodes[toNode].y;
  globalHeading = poseNodes[toNode].headingDeg;
}

// Compares the newest node against earlier non-adjacent nodes; if one is
// close enough to plausibly be a revisit, runs ICP against it and accepts
// the loop closure only if the fix is modest (a huge fix usually means ICP
// latched onto the wrong wall, not an honest revisit).
void checkLoopClosure(int newNodeIdx) {
  PoseNode &newNode = poseNodes[newNodeIdx];

  for (int i = 0; i < newNodeIdx - LOOP_CLOSURE_MIN_NODE_GAP; i++) {
    PoseNode &old = poseNodes[i];
    float d = sqrt(sq(newNode.x - old.x) + sq(newNode.y - old.y));
    if (d > LOOP_CLOSURE_DIST_MM) continue;

    float srcX[80], srcY[80];
    int n = min(newNode.scanCount, 80);
    for (int k = 0; k < n; k++) {
      srcX[k] = scanPoints[newNode.scanStartIdx + k].x;
      srcY[k] = scanPoints[newNode.scanStartIdx + k].y;
    }
    float refX[80], refY[80];
    int rn = min(old.scanCount, 80);
    for (int k = 0; k < rn; k++) {
      refX[k] = scanPoints[old.scanStartIdx + k].x;
      refY[k] = scanPoints[old.scanStartIdx + k].y;
    }

    Transform2D loopFix = icpAlignToReference(srcX, srcY, n, refX, refY, rn, 0);
    float fixMag = sqrt(loopFix.dx * loopFix.dx + loopFix.dy * loopFix.dy);

    bool loopFixTrustworthy = (icpLastInlierCount >= ICP_MIN_INLIERS) &&
                               (fixMag <= ICP_MAX_CORRECTION_MM) &&
                               (fabs(loopFix.dTheta) <= ICP_MAX_CORRECTION_DEG);

    if (loopFixTrustworthy) {
      if (poseEdgeCount < MAX_POSE_NODES * 2) {
        poseEdges[poseEdgeCount] = {i, newNodeIdx, loopFix.dx, loopFix.dy, loopFix.dTheta};
        poseEdgeCount++;
      }
      distributeLoopCorrection(i, newNodeIdx, loopFix);
      break;
    }
  }
}

// Called once per completed 360 scan rotation. Registers the new scan
// against the previous node via ICP, corrects its world-frame points in
// place, records the node + sequential edge, then checks for loop closure.
void finalizeScanNode(int scanStartIdx, int scanEndIdxExclusive) {
  if (poseNodeCount >= MAX_POSE_NODES) return;

  int newCount = scanEndIdxExclusive - scanStartIdx;
  if (newCount <= 0) return;

  if (poseNodeCount > 0) {
    PoseNode &prev = poseNodes[poseNodeCount - 1];

    float srcX[80], srcY[80];
    int n = min(newCount, 80);
    for (int i = 0; i < n; i++) {
      srcX[i] = scanPoints[scanStartIdx + i].x;
      srcY[i] = scanPoints[scanStartIdx + i].y;
    }
    float refX[80], refY[80];
    int rn = min(prev.scanCount, 80);
    for (int i = 0; i < rn; i++) {
      refX[i] = scanPoints[prev.scanStartIdx + i].x;
      refY[i] = scanPoints[prev.scanStartIdx + i].y;
    }

    Transform2D correction = icpAlignToReference(srcX, srcY, n, refX, refY, rn, 0);

    float correctionMag = sqrt(correction.dx * correction.dx + correction.dy * correction.dy);
    bool correctionTrustworthy = (icpLastInlierCount >= ICP_MIN_INLIERS) &&
                                  (correctionMag <= ICP_MAX_CORRECTION_MM) &&
                                  (fabs(correction.dTheta) <= ICP_MAX_CORRECTION_DEG);

    if (correctionTrustworthy) {
      float cosT = cos(correction.dTheta * PI / 180.0), sinT = sin(correction.dTheta * PI / 180.0);
      for (int i = scanStartIdx; i < scanEndIdxExclusive; i++) {
        float rx = cosT * scanPoints[i].x - sinT * scanPoints[i].y + correction.dx;
        float ry = sinT * scanPoints[i].x + cosT * scanPoints[i].y + correction.dy;
        scanPoints[i].x = rx;
        scanPoints[i].y = ry;
      }

      if (poseEdgeCount < MAX_POSE_NODES * 2) {
        poseEdges[poseEdgeCount] = {poseNodeCount - 1, poseNodeCount, correction.dx, correction.dy, correction.dTheta};
        poseEdgeCount++;
      }
    }
    // else: low-confidence fit (too few inliers or implausible correction) -
    // leave this scan's points exactly as raw odometry placed them, rather
    // than warping them onto the wrong wall. This is what was causing the
    // kinked/rotated section in your scan.
  }

  PoseNode &node = poseNodes[poseNodeCount];
  node.x = botX;
  node.y = botY;
  node.headingDeg = globalHeading;
  node.scanStartIdx = scanStartIdx;
  node.scanCount = newCount;
  poseNodeCount++;

  checkLoopClosure(poseNodeCount - 1);

  // Point-cloud post-processing runs here, after registration/loop-closure
  // is fully resolved, so it always works from final corrected geometry.
  buildCleanPolylines();
}

// =========================================================================
// POINT CLOUD POST-PROCESSING PIPELINE
// =========================================================================
// Pipeline: gather all corrected world-frame scan points -> Statistical
// Outlier Removal (SOR) -> Radius Outlier Removal (ROR) -> Moving Least
// Squares (MLS) smoothing -> DBSCAN clustering -> nearest-neighbor path
// ordering per cluster -> Chaikin corner-cutting smoothing -> clean
// polylines. No straight-line fitting anywhere - curved/irregular walls
// are preserved as smoothed point sequences.

#define MAX_CLEAN_POINTS   400   // matches MAX_SCAN_POINTS

// --- SOR (Statistical Outlier Removal) ---
#define SOR_K_NEIGHBORS       8
#define SOR_STD_MULTIPLIER  1.5

// --- ROR (Radius Outlier Removal) ---
#define ROR_RADIUS_MM      150.0
#define ROR_MIN_NEIGHBORS     3

// --- MLS (Moving Least Squares) smoothing ---
#define MLS_RADIUS_MM      120.0

// --- DBSCAN clustering ---
#define DBSCAN_EPS_MM      150.0
#define DBSCAN_MIN_PTS        4
#define MAX_CLUSTERS          12

// --- Path ordering + Chaikin smoothing ---
#define MAX_POLYLINE_POINTS  MAX_POLYLINE_POINTS_FWD  // kept as one name everywhere below; the _FWD one exists solely so Polyline can be declared near the top of the file
#define CHAIKIN_ITERATIONS     2

CleanPoint pcRaw[MAX_CLEAN_POINTS];
int pcRawCount = 0;
CleanPoint pcSOR[MAX_CLEAN_POINTS];
int pcSORCount = 0;
CleanPoint pcROR[MAX_CLEAN_POINTS];
int pcRORCount = 0;
CleanPoint pcMLS[MAX_CLEAN_POINTS];
int pcMLSCount = 0;

int dbscanLabels[MAX_CLEAN_POINTS];  // -2 = unvisited, -1 = noise, >=0 = cluster id

Polyline cleanPolylines[MAX_CLUSTERS];
int cleanPolylineCount = 0;

float pcDistSq(const CleanPoint &a, const CleanPoint &b) {
  float dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Gathers every currently-valid scan point (already in corrected world
// frame, post ICP/loop-closure) into pcRaw[].
void gatherValidScanPoints() {
  pcRawCount = 0;
  for (int i = 0; i < scanPointCount && pcRawCount < MAX_CLEAN_POINTS; i++) {
    if (scanPoints[i].valid) {
      pcRaw[pcRawCount].x = scanPoints[i].x;
      pcRaw[pcRawCount].y = scanPoints[i].y;
      pcRawCount++;
    }
  }
}

// Statistical Outlier Removal: for each point, computes the mean distance
// to its K nearest neighbors, then discards points whose mean neighbor
// distance is more than SOR_STD_MULTIPLIER standard deviations above the
// global average - removes floating noise specks.
void runSOR(CleanPoint* in, int inCount, CleanPoint* out, int* outCount) {
  static float meanDist[MAX_CLEAN_POINTS];

  for (int i = 0; i < inCount; i++) {
    float kBest[SOR_K_NEIGHBORS];
    int kFound = 0;
    for (int k = 0; k < SOR_K_NEIGHBORS; k++) kBest[k] = 1e12;

    for (int j = 0; j < inCount; j++) {
      if (j == i) continue;
      float d = sqrt(pcDistSq(in[i], in[j]));
      if (d < kBest[SOR_K_NEIGHBORS - 1]) {
        int pos = SOR_K_NEIGHBORS - 1;
        kBest[pos] = d;
        while (pos > 0 && kBest[pos - 1] > kBest[pos]) {
          float t = kBest[pos - 1]; kBest[pos - 1] = kBest[pos]; kBest[pos] = t;
          pos--;
        }
        if (kFound < SOR_K_NEIGHBORS) kFound++;
      }
    }

    float sum = 0;
    int usedK = min(kFound, SOR_K_NEIGHBORS);
    for (int k = 0; k < usedK; k++) sum += kBest[k];
    meanDist[i] = (usedK > 0) ? (sum / usedK) : 0;
  }

  float globalMean = 0;
  for (int i = 0; i < inCount; i++) globalMean += meanDist[i];
  globalMean /= max(inCount, 1);

  float variance = 0;
  for (int i = 0; i < inCount; i++) {
    float d = meanDist[i] - globalMean;
    variance += d * d;
  }
  variance /= max(inCount, 1);
  float stdDev = sqrt(variance);
  float threshold = globalMean + SOR_STD_MULTIPLIER * stdDev;

  *outCount = 0;
  for (int i = 0; i < inCount; i++) {
    if (meanDist[i] <= threshold && *outCount < MAX_CLEAN_POINTS) {
      out[*outCount] = in[i];
      (*outCount)++;
    }
  }
}

// Radius Outlier Removal: discards any point with fewer than
// ROR_MIN_NEIGHBORS other points within ROR_RADIUS_MM.
void runROR(CleanPoint* in, int inCount, CleanPoint* out, int* outCount) {
  float radiusSq = ROR_RADIUS_MM * ROR_RADIUS_MM;
  *outCount = 0;
  for (int i = 0; i < inCount; i++) {
    int neighborCount = 0;
    for (int j = 0; j < inCount; j++) {
      if (j == i) continue;
      if (pcDistSq(in[i], in[j]) <= radiusSq) neighborCount++;
      if (neighborCount >= ROR_MIN_NEIGHBORS) break;
    }
    if (neighborCount >= ROR_MIN_NEIGHBORS && *outCount < MAX_CLEAN_POINTS) {
      out[*outCount] = in[i];
      (*outCount)++;
    }
  }
}

// Moving Least Squares smoothing: replaces each point with a
// distance-weighted (Gaussian) average of every point within
// MLS_RADIUS_MM, pulling points toward the local mean surface while
// preserving the wall's overall curved shape.
void runMLS(CleanPoint* in, int inCount, CleanPoint* out, int* outCount) {
  float radiusSq = MLS_RADIUS_MM * MLS_RADIUS_MM;
  float sigma = MLS_RADIUS_MM / 2.0;

  *outCount = 0;
  for (int i = 0; i < inCount; i++) {
    float sumX = 0, sumY = 0, sumW = 0;
    for (int j = 0; j < inCount; j++) {
      float dSq = pcDistSq(in[i], in[j]);
      if (dSq > radiusSq) continue;
      float w = exp(-dSq / (2.0 * sigma * sigma));
      sumX += in[j].x * w;
      sumY += in[j].y * w;
      sumW += w;
    }
    if (sumW > 0 && *outCount < MAX_CLEAN_POINTS) {
      out[*outCount].x = sumX / sumW;
      out[*outCount].y = sumY / sumW;
      (*outCount)++;
    }
  }
}

int dbscanRegionQuery(CleanPoint* pts, int count, int idx, int* neighborsOut) {
  float epsSq = DBSCAN_EPS_MM * DBSCAN_EPS_MM;
  int n = 0;
  for (int j = 0; j < count; j++) {
    if (pcDistSq(pts[idx], pts[j]) <= epsSq) neighborsOut[n++] = j;
  }
  return n;
}

// Standard DBSCAN. Fills dbscanLabels[] with cluster ids (0..N-1) or -1
// for noise.
int runDBSCAN(CleanPoint* pts, int count) {
  for (int i = 0; i < count; i++) dbscanLabels[i] = -2;

  static int neighbors[MAX_CLEAN_POINTS];
  static int seedSet[MAX_CLEAN_POINTS];
  int clusterId = 0;

  for (int i = 0; i < count; i++) {
    if (dbscanLabels[i] != -2) continue;

    int nCount = dbscanRegionQuery(pts, count, i, neighbors);
    if (nCount < DBSCAN_MIN_PTS) {
      dbscanLabels[i] = -1;
      continue;
    }

    dbscanLabels[i] = clusterId;
    int seedCount = 0;
    for (int k = 0; k < nCount; k++) seedSet[seedCount++] = neighbors[k];

    for (int s = 0; s < seedCount; s++) {
      int qIdx = seedSet[s];
      if (dbscanLabels[qIdx] == -1) dbscanLabels[qIdx] = clusterId;
      if (dbscanLabels[qIdx] != -2) continue;

      dbscanLabels[qIdx] = clusterId;
      int qNCount = dbscanRegionQuery(pts, count, qIdx, neighbors);
      if (qNCount >= DBSCAN_MIN_PTS) {
        for (int k = 0; k < qNCount && seedCount < MAX_CLEAN_POINTS; k++) {
          seedSet[seedCount++] = neighbors[k];
        }
      }
    }

    clusterId++;
    if (clusterId >= MAX_CLUSTERS) break;
  }

  return clusterId;
}

// Greedy nearest-neighbor path ordering: starts at the cluster's leftmost
// point and repeatedly hops to the nearest unvisited point, building one
// continuous path through the cluster.
void orderClusterNearestNeighbor(CleanPoint* clusterPts, int clusterCount, CleanPoint* orderedOut, int* orderedCount) {
  static bool used[MAX_CLEAN_POINTS];
  int n = min(clusterCount, MAX_POLYLINE_POINTS);
  for (int i = 0; i < n; i++) used[i] = false;

  int startIdx = 0;
  for (int i = 1; i < n; i++) {
    if (clusterPts[i].x < clusterPts[startIdx].x) startIdx = i;
  }

  int current = startIdx;
  used[current] = true;
  orderedOut[0] = clusterPts[current];
  *orderedCount = 1;

  for (int step = 1; step < n; step++) {
    float bestDistSq = 1e18;
    int bestIdx = -1;
    for (int j = 0; j < n; j++) {
      if (used[j]) continue;
      float d = pcDistSq(clusterPts[current], clusterPts[j]);
      if (d < bestDistSq) { bestDistSq = d; bestIdx = j; }
    }
    if (bestIdx < 0) break;
    used[bestIdx] = true;
    current = bestIdx;
    orderedOut[*orderedCount] = clusterPts[current];
    (*orderedCount)++;
  }
}

// Chaikin's corner-cutting: each interior edge is replaced by two points
// at 1/4 and 3/4 along it, smoothing the curve while keeping the original
// start/end points fixed. No straight segments are fitted - output stays
// a dense smoothed point sequence.
void chaikinSmooth(CleanPoint* in, int inCount, CleanPoint* out, int* outCount) {
  if (inCount < 3) {
    for (int i = 0; i < inCount; i++) out[i] = in[i];
    *outCount = inCount;
    return;
  }

  static CleanPoint buf[MAX_POLYLINE_POINTS];
  int bufCount = inCount;
  for (int i = 0; i < inCount; i++) buf[i] = in[i];

  for (int iter = 0; iter < CHAIKIN_ITERATIONS; iter++) {
    int newCount = 2 * (bufCount - 1);
    if (newCount + 2 > MAX_POLYLINE_POINTS) break;

    static CleanPoint next[MAX_POLYLINE_POINTS];
    int n = 0;
    next[n++] = buf[0];
    for (int i = 0; i < bufCount - 1; i++) {
      CleanPoint &p0 = buf[i];
      CleanPoint &p1 = buf[i + 1];
      CleanPoint q = { 0.75f * p0.x + 0.25f * p1.x, 0.75f * p0.y + 0.25f * p1.y };
      CleanPoint r = { 0.25f * p0.x + 0.75f * p1.x, 0.25f * p0.y + 0.75f * p1.y };
      next[n++] = q;
      next[n++] = r;
    }
    next[n++] = buf[bufCount - 1];

    bufCount = n;
    for (int i = 0; i < bufCount; i++) buf[i] = next[i];
  }

  *outCount = min(bufCount, MAX_POLYLINE_POINTS);
  for (int i = 0; i < *outCount; i++) out[i] = buf[i];
}

// Orchestrates the full pipeline. Called once per completed 360 scan.
void buildCleanPolylines() {
  gatherValidScanPoints();
  runSOR(pcRaw, pcRawCount, pcSOR, &pcSORCount);
  runROR(pcSOR, pcSORCount, pcROR, &pcRORCount);
  runMLS(pcROR, pcRORCount, pcMLS, &pcMLSCount);

  cleanPolylineCount = 0;
  if (pcMLSCount < DBSCAN_MIN_PTS) return;

  int numClusters = runDBSCAN(pcMLS, pcMLSCount);

  static CleanPoint clusterBuf[MAX_CLEAN_POINTS];
  static CleanPoint orderedBuf[MAX_POLYLINE_POINTS];

  for (int c = 0; c < numClusters && cleanPolylineCount < MAX_CLUSTERS; c++) {
    int clusterCount = 0;
    for (int i = 0; i < pcMLSCount && clusterCount < MAX_CLEAN_POINTS; i++) {
      if (dbscanLabels[i] == c) clusterBuf[clusterCount++] = pcMLS[i];
    }
    if (clusterCount < DBSCAN_MIN_PTS) continue;

    int orderedCount = 0;
    orderClusterNearestNeighbor(clusterBuf, clusterCount, orderedBuf, &orderedCount);

    Polyline &poly = cleanPolylines[cleanPolylineCount];
    chaikinSmooth(orderedBuf, orderedCount, poly.pts, &poly.count);
    cleanPolylineCount++;
  }
}

// =========================================================================
// INTERRUPT SERVICE ROUTINES
// =========================================================================

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_C2) == HIGH) leftTicks--;
  else leftTicks++;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_C2) == HIGH) rightTicks++;
  else rightTicks--;
}

// Set by the MPU6050 whenever a fresh DMP packet is ready, so loop() only
// bothers reading over I2C when there's actually new data.
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

// =========================================================================
// DASHBOARD (served at "/")
// =========================================================================
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MappingBot</title>
<style>
  :root {
    --bg: #14161b;
    --panel-bg: #1a1d24;
    --card-bg: #20232b;
    --card-bg-2: #262a33;
    --border: #2d313b;
    --text: #e8e9ec;
    --muted: #8a90a0;
    --blue: #3498db;
    --blue-dark: #2980b9;
    --green: #2ecc71;
    --green-dark: #27ae60;
    --red: #e74c3c;
    --red-dark: #c0392b;
    --purple: #9b59b6;
    --purple-dark: #8e44ad;
    --amber: #f1c40f;
  }
  * { box-sizing: border-box; }
  body {
    font-family: -apple-system, Segoe UI, Roboto, sans-serif;
    background: var(--bg); margin: 0; padding: 0;
    color: var(--text);
  }
  h2 { margin: 0; letter-spacing: 0.5px; font-size: 18px; }
  h3 {
    margin: 0 0 12px; font-size: 13px; font-weight: 700;
    letter-spacing: 1px; text-transform: uppercase; color: var(--muted);
    text-align: left;
  }

  .app { display: flex; min-height: 100vh; }

  /* ---- sidebar ---- */
  .sidebar {
    width: 190px; flex-shrink: 0; background: var(--panel-bg);
    border-right: 1px solid var(--border);
    padding: 18px 12px; display: flex; flex-direction: column; gap: 6px;
  }
  .brand {
    padding: 4px 8px 18px; border-bottom: 1px solid var(--border);
    margin-bottom: 10px;
  }
  .battInfo {
    font-size: 12px; color: var(--muted); margin-top: 6px;
  }
  .battInfo.low { color: var(--red); font-weight: 600; }
  .navBtn {
    display: flex; align-items: center; gap: 10px;
    background: transparent; color: var(--muted); border: none;
    padding: 11px 12px; border-radius: 10px; font-size: 14px;
    text-align: left; cursor: pointer; user-select: none;
  }
  .navBtn .ic { font-size: 16px; width: 20px; text-align: center; }
  .navBtn:active { background: var(--card-bg-2); }
  .navBtn.active { background: var(--blue); color: #fff; }

  /* ---- content ---- */
  .content { flex: 1; padding: 16px 12px 40px; max-width: 460px; margin: 0 auto; text-align: center; }

  .card {
    background: var(--card-bg); border: 1px solid var(--border);
    border-radius: 16px; padding: 16px;
    margin: 0 0 14px;
    box-shadow: 0 2px 10px rgba(0,0,0,0.25);
  }

  .panel { display: none; }
  .panel.active { display: block; }

  /* ---- status readout (always visible) ---- */
  .statRow { display: flex; gap: 8px; }
  .statBox { flex: 1; background: var(--card-bg-2); border-radius: 10px; padding: 8px 4px; }
  .statBox .label { font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px; }
  .statBox .value { font-size: 17px; font-weight: 600; margin-top: 2px; }
  #stateVal { color: var(--muted); }

  /* ---- generic buttons ---- */
  .note { font-size: 12px; color: var(--muted); margin: 0 0 10px; text-align: left; }
  .row { display: flex; justify-content: center; gap: 8px; margin-bottom: 8px; flex-wrap: wrap; }
  .btn {
    padding: 12px 14px; border-radius: 10px;
    background: var(--blue); color: white; font-size: 14px;
    border: none; user-select: none;
  }
  .btn:active { background: var(--blue-dark); }
  .btn:disabled { background: #33363f; color: #6c7180; }
  .btn.mark { background: var(--purple); }
  .btn.mark:active { background: var(--purple-dark); }
  .btn.save { background: var(--green); }
  .btn.save:active { background: var(--green-dark); }
  .btn.stop { background: var(--red); }
  .btn.stop:active { background: var(--red-dark); }
  .btn.ghost { background: var(--card-bg-2); color: var(--text); border: 1px solid var(--border); }
  .btn.ghost:active { background: #2f333d; }
  #calibInfo, #pathInfo, #tuneMsg { font-size: 13px; color: var(--muted); margin-top: 10px; white-space: pre-line; text-align: left; }

  /* ---- D-pad ---- */
  .dpad {
    display: grid; grid-template-columns: 76px 76px 76px;
    grid-template-rows: 76px 76px 76px;
    gap: 8px; justify-content: center; margin: 4px auto 0;
  }
  .dpadBtn {
    border: none; border-radius: 14px; background: var(--green); color: white;
    font-size: 26px; user-select: none; touch-action: none;
    display: flex; align-items: center; justify-content: center;
  }
  .dpadBtn:active { background: var(--green-dark); }
  .dpadBtn:disabled { background: #33363f; color: #6c7180; }
  #dpadFwd  { grid-column: 2; grid-row: 1; }
  #dpadLeft { grid-column: 1; grid-row: 2; }
  #dpadRight{ grid-column: 3; grid-row: 2; }
  #dpadBwd  { grid-column: 2; grid-row: 3; }

  .canvasViewport {
    width: 400px; height: 400px; max-width: 100%; aspect-ratio: 1 / 1;
    overflow: hidden; position: relative;
    border: 2px solid var(--border); border-radius: 10px;
    background: #101216; touch-action: none;
  }
  #pathCanvas {
    position: absolute; top: 0; left: 0;
    cursor: grab;
  }
  #pathCanvas.dragging { cursor: grabbing; }

  /* ---- Tune panel ---- */
  .tuneGroup { text-align: left; margin-bottom: 14px; }
  .tuneGroup h4 {
    margin: 0 0 8px; font-size: 12px; color: var(--muted);
    text-transform: uppercase; letter-spacing: 0.5px;
  }
  .tuneField {
    display: flex; align-items: center; justify-content: space-between;
    gap: 10px; padding: 7px 0; border-bottom: 1px solid var(--border);
  }
  .tuneField:last-child { border-bottom: none; }
  .tuneField label:not(.switch) { font-size: 13px; color: var(--text); flex: 1; }
  .tuneField input[type=number] {
    width: 90px; background: var(--card-bg-2); border: 1px solid var(--border);
    color: var(--text); border-radius: 8px; padding: 6px 8px; font-size: 13px;
    text-align: right;
  }

  /* boolean toggle switches */
  .switch { position: relative; display: inline-block; width: 42px; height: 24px; flex-shrink: 0; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider {
    position: absolute; cursor: pointer; inset: 0;
    background: #3a3e49; border-radius: 24px; transition: .15s;
  }
  .slider:before {
    position: absolute; content: ""; height: 18px; width: 18px;
    left: 3px; bottom: 3px; background: #cfd2d9; border-radius: 50%; transition: .15s;
  }
  .switch input:checked + .slider { background: var(--blue); }
  .switch input:checked + .slider:before { transform: translateX(18px); background: #fff; }
</style>
</head>
<body>
<div class="app">

  <nav class="sidebar">
    <div class="brand">
      <h2>MappingBot</h2>
      <div class="battInfo" id="battInfo">C1 -- V &nbsp;|&nbsp; C2 -- V</div>
    </div>
    <button class="navBtn active" data-panel="manual"><span class="ic">&#127918;</span>Manual</button>
    <button class="navBtn" data-panel="auto"><span class="ic">&#128506;</span>Automatic</button>
    <button class="navBtn" data-panel="calib"><span class="ic">&#129517;</span>Calibration</button>
    <button class="navBtn" data-panel="tune"><span class="ic">&#128295;</span>Tune</button>
  </nav>

  <main class="content">

    <div class="card">
      <div class="statRow">
        <div class="statBox"><div class="label">Distance</div><div class="value" id="distVal">0 mm</div></div>
        <div class="statBox"><div class="label">Heading</div><div class="value" id="headingVal">0.0&deg;</div></div>
        <div class="statBox"><div class="label">State</div><div class="value" id="stateVal">idle</div></div>
      </div>
    </div>

    <!-- ============== MANUAL PANEL ============== -->
    <section id="panel-manual" class="panel active">
      <div class="card">
        <h3>Manual Drive</h3>
        <div class="dpad">
          <button class="dpadBtn" id="dpadFwd" data-dir="fwd">&#9650;</button>
          <button class="dpadBtn" id="dpadLeft" data-dir="left">&#9664;</button>
          <button class="dpadBtn" id="dpadRight" data-dir="right">&#9654;</button>
          <button class="dpadBtn" id="dpadBwd" data-dir="bwd">&#9660;</button>
        </div>
      </div>

      <div class="card">
        <h3>Turns</h3>
        <div class="row">
          <button class="btn turnBtn" onclick="turn('ccw',90)">CCW 90</button>
          <button class="btn turnBtn" onclick="turn('ccw',180)">CCW 180</button>
          <button class="btn turnBtn" onclick="turn('ccw',270)">CCW 270</button>
          <button class="btn turnBtn" onclick="turn('ccw',360)">CCW 360</button>
        </div>
        <div class="row">
          <button class="btn turnBtn" onclick="turn('cw',90)">CW 90</button>
          <button class="btn turnBtn" onclick="turn('cw',180)">CW 180</button>
          <button class="btn turnBtn" onclick="turn('cw',270)">CW 270</button>
          <button class="btn turnBtn" onclick="turn('cw',360)">CW 360</button>
        </div>
        <button class="btn stop" style="width:100%" onclick="fetch('/stop')">STOP</button>
      </div>
    </section>

    <!-- ============== AUTOMATIC PANEL ============== -->
    <section id="panel-auto" class="panel">
      <div class="card">
        <h3>Path Planning</h3>
        <p class="note">Each square = 10cm. Tap grid points to draw a route (in order), then Send &amp; Run. The red triangle shows the bot's live position and heading. Amber dots show the last 360&deg; obstacle scan.</p>
        <div class="canvasViewport">
          <canvas id="pathCanvas" width="4000" height="4000"></canvas>
        </div>
        <div class="row" style="margin-top:10px;">
          <button class="btn pathBtn" onclick="undoPoint()">Undo point</button>
          <button class="btn pathBtn" onclick="clearPath()">Clear path</button>
          <button class="btn pathBtn" onclick="recenterOnBot()">Center</button>
        </div>
        <div class="row">
          <button class="btn save pathBtn" onclick="sendPath()">Send &amp; Run path</button>
          <button class="btn stop" onclick="fetch('/path/stop')">Stop path</button>
        </div>
        <div class="row">
          <button class="btn ghost pathBtn" onclick="fetch('/path/pause')">Pause</button>
          <button class="btn ghost pathBtn" onclick="fetch('/path/resume')">Resume</button>
        </div>
        <div class="row">
          <button class="btn pathBtn" onclick="resetPosition()">Mark here as start (0,0)</button>
        </div>
        <div id="pathInfo">No path drawn yet</div>
      </div>

      <div class="card">
        <h3>360&deg; Obstacle Scan</h3>
        <p class="note">The bot rotates a full circle in place, taking ToF distance readings every few degrees, and plots them as obstacle points on the map above.</p>
        <div class="row">
          <button class="btn save scanBtn" onclick="startScan()">Start 360&deg; Scan</button>
          <button class="btn stop" onclick="fetch('/scan/stop')">Stop scan</button>
        </div>
        <div class="row">
          <button class="btn ghost scanBtn" onclick="clearScan()">Clear scan map</button>
        </div>
        <div id="scanInfo">No scan yet</div>
      </div>
    </section>

    <!-- ============== CALIBRATION PANEL ============== -->
    <section id="panel-calib" class="panel">
      <div class="card">
        <h3>Gyro Turn Calibration</h3>
        <p class="note">Tap Start, rotate the bot BY HAND to roughly the angle shown, tap the matching Mark button, repeat a few times, then Save.</p>
        <div class="row">
          <button class="btn calibBtn" onclick="calibStart()">Start</button>
          <button class="btn ghost calibBtn" onclick="fetch('/calib/stop').then(refreshCalib)">Cancel</button>
        </div>
        <div class="row">
          <button class="btn mark calibBtn" onclick="calibMark(90)">Mark 90</button>
          <button class="btn mark calibBtn" onclick="calibMark(180)">Mark 180</button>
          <button class="btn mark calibBtn" onclick="calibMark(270)">Mark 270</button>
          <button class="btn mark calibBtn" onclick="calibMark(360)">Mark 360</button>
        </div>
        <div class="row">
          <button class="btn save calibBtn" onclick="fetch('/calib/save').then(refreshCalib)">Save to EEPROM</button>
          <button class="btn ghost calibBtn" onclick="fetch('/rebias')">Rebias gyro</button>
        </div>
        <div id="calibInfo">Not calibrating.</div>
      </div>
    </section>

    <!-- ============== TUNE PANEL ============== -->
    <section id="panel-tune" class="panel">
      <div class="card">
        <h3>Tune Parameters</h3>
        <p class="note">Apply changes instantly (RAM only) or Save to EEPROM to keep them after reboot. Blocked while the bot is moving/turning/calibrating/following a path/scanning.</p>

        <div class="tuneGroup">
          <h4>Speed</h4>
          <div class="tuneField"><label>Base speed (0-255)</label><input type="number" id="tBaseSpeed" step="1"></div>
          <div class="tuneField"><label>Left speed scale</label><input type="number" id="tLeftScale" step="0.01"></div>
          <div class="tuneField"><label>Right speed scale</label><input type="number" id="tRightScale" step="0.01"></div>
        </div>

        <div class="tuneGroup">
          <h4>Wheels / Encoders</h4>
          <div class="tuneField"><label>Left ticks / rev</label><input type="number" id="tLeftTicks" step="0.1"></div>
          <div class="tuneField"><label>Right ticks / rev</label><input type="number" id="tRightTicks" step="0.1"></div>
          <div class="tuneField"><label>Wheel diameter (mm)</label><input type="number" id="tWheelDia" step="0.1"></div>
        </div>

        <div class="tuneGroup">
          <h4>Drive-straight correction</h4>
          <div class="tuneField"><label>Heading Kp</label><input type="number" id="tHeadingKP" step="0.1"></div>
        </div>

        <div class="tuneGroup">
          <h4>Turning</h4>
          <div class="tuneField"><label>Turn speed max (0-255)</label><input type="number" id="tTurnMax" step="1"></div>
          <div class="tuneField"><label>Turn speed min (0-255)</label><input type="number" id="tTurnMin" step="1"></div>
          <div class="tuneField"><label>Slowdown start (deg)</label><input type="number" id="tTurnSlow" step="1"></div>
          <div class="tuneField"><label>Tolerance (deg)</label><input type="number" id="tTurnTol" step="0.1"></div>
          <div class="tuneField"><label>Coast lead (deg)</label><input type="number" id="tTurnLead" step="0.1"></div>
        </div>

        <div class="tuneGroup">
          <h4>Path following</h4>
          <div class="tuneField"><label>Arrive tolerance (mm)</label><input type="number" id="tPathTol" step="0.5"></div>
          <div class="tuneField">
            <label>Reverse path turn direction</label>
            <label class="switch"><input type="checkbox" id="tPathRev"><span class="slider"></span></label>
          </div>
        </div>

        <div class="tuneGroup">
          <h4>360&deg; scan</h4>
          <div class="tuneField"><label>Step size (deg)</label><input type="number" id="tScanStep" step="0.5"></div>
          <div class="tuneField"><label>Settle time (ms)</label><input type="number" id="tScanSettle" step="5"></div>
        </div>

        <div class="tuneGroup">
          <h4>Battery protection</h4>
          <div class="tuneField"><label>Min cell voltage (V)</label><input type="number" id="tMinCell" step="0.01"></div>
        </div>
        <div class="tuneGroup">
          <h4>Motor direction inversion</h4>
          <div class="tuneField">
            <label>Invert left motor</label>
            <label class="switch"><input type="checkbox" id="tInvLeft"><span class="slider"></span></label>
          </div>
          <div class="tuneField">
            <label>Invert right motor</label>
            <label class="switch"><input type="checkbox" id="tInvRight"><span class="slider"></span></label>
          </div>
          <div class="tuneField">
            <label>Invert turn direction</label>
            <label class="switch"><input type="checkbox" id="tInvTurn"><span class="slider"></span></label>
          </div>
        </div>

        <div class="row">
          <button class="btn tuneBtn" onclick="applyTune()">Apply</button>
          <button class="btn save tuneBtn" onclick="saveTune()">Save to EEPROM</button>
          <button class="btn ghost tuneBtn" onclick="resetTune()">Reset to defaults</button>
        </div>
        <div id="tuneMsg"></div>
      </div>
    </section>

  </main>
</div>

<script>
const distEl = document.getElementById('distVal');
const headingEl = document.getElementById('headingVal');
const stateEl = document.getElementById('stateVal');
const calibInfo = document.getElementById('calibInfo');
const turnBtns = document.querySelectorAll('.turnBtn');
const calibBtns = document.querySelectorAll('.calibBtn');
const pathBtns = document.querySelectorAll('.pathBtn');
const scanBtns = document.querySelectorAll('.scanBtn');
const dpadBtns = document.querySelectorAll('.dpadBtn');
const tuneBtns = document.querySelectorAll('.tuneBtn');
const pathInfo = document.getElementById('pathInfo');
const scanInfo = document.getElementById('scanInfo');
const tuneMsg = document.getElementById('tuneMsg');
let pingInterval = null;

// ---------------- Sidebar nav ----------------
document.querySelectorAll('.navBtn').forEach(b => {
  b.addEventListener('click', () => {
    document.querySelectorAll('.navBtn').forEach(x => x.classList.remove('active'));
    b.classList.add('active');
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    document.getElementById('panel-' + b.dataset.panel).classList.add('active');
    if (b.dataset.panel === 'tune') fetchTune();
  });
});

// ---------------- Battery readout (sidebar) ----------------
const minCellVoltageWarn = 3.3; // just for the red-text warning in the sidebar; the real cutoff lives in Tune
function fetchBattery() {
  fetch('/status').then(r => r.json()).then(d => {
    const el = document.getElementById('battInfo');
    el.innerText = `C1 ${d.cell1.toFixed(2)}V  |  C2 ${d.cell2.toFixed(2)}V`;
    el.classList.toggle('low', d.cell1 < minCellVoltageWarn || d.cell2 < minCellVoltageWarn);
  }).catch(() => {});
}
fetchBattery();
setInterval(fetchBattery, 10000);

function moveStart(dir) {
  fetch(`/move?dir=${dir}`);
  pingInterval = setInterval(() => fetch('/ping'), 150);
}
function moveStop() {
  fetch('/stop');
  clearInterval(pingInterval);
}
function turn(dir, deg) { fetch(`/turn?dir=${dir}&deg=${deg}`); }
function calibStart() { fetch('/calib/start').then(refreshCalib); }
function calibMark(deg) {
  fetch(`/calib/mark?deg=${deg}`).then(r => r.json()).then(updateCalibInfo);
}
function refreshCalib() { fetch('/calib/status').then(r => r.json()).then(updateCalibInfo); }

function updateCalibInfo(d) {
  if (!d.calibrating) {
    calibInfo.innerText = `Not calibrating. Saved scale: ${d.scale.toFixed(3)}`;
  } else {
    calibInfo.innerText = `Calibrating. Raw heading: ${d.rawHeading.toFixed(1)} deg. Last mark scale: ${d.lastMarkScale.toFixed(3)}`;
  }
}

dpadBtns.forEach(b => {
  const dir = b.dataset.dir;
  b.addEventListener('pointerdown', (e) => { e.preventDefault(); moveStart(dir); });
  b.addEventListener('pointerup', moveStop);
  b.addEventListener('pointerleave', moveStop);
  b.addEventListener('pointercancel', moveStop);
});

// ---------------- Path planning canvas ----------------
const canvas = document.getElementById('pathCanvas');
const ctx = canvas.getContext('2d');
const CELL_PX = 25;       // pixels per grid cell on screen (unchanged)
const GRID_N = 160;       // 10x the old 16 -> 10x bigger playfield
const CELL_MM = 100;      // 10cm per cell, must match GRID_CELL_MM on the firmware
const centerIdx = GRID_N / 2;
const scalePxPerMM = CELL_PX / CELL_MM;
const VIEWPORT_PX = 400;  // matches .canvasViewport's fixed size

let path = [];          // array of {x, y} in mm, world frame, in click order (free placement, not grid-snapped)
let pathOrigin = { x: 0, y: 0 };  // bot position captured ONCE when the first point of a new path is placed
let botPos = { x: 0, y: 0, heading: 0 }; // live from /status, in mm/deg
let scanPoints = [];    // array of {x, y, valid} from /scan/points, in mm world frame
let cleanPolylines = []; // array of {points:[{x,y},...]} from /scan/polylines - post-processed smoothed walls

// ---------------- Pan state ----------------
let panX = 0, panY = 0;
function clampPan() {
  const canvasPx = GRID_N * CELL_PX;
  const minPan = VIEWPORT_PX - canvasPx;
  panX = Math.min(0, Math.max(minPan, panX));
  panY = Math.min(0, Math.max(minPan, panY));
}
function applyPan() {
  canvas.style.transform = `translate(${panX}px, ${panY}px)`;
}
// start centered on the world origin
panX = VIEWPORT_PX / 2 - centerIdx * CELL_PX;
panY = VIEWPORT_PX / 2 - centerIdx * CELL_PX;
clampPan();
applyPan();

let isPanning = false, dragMoved = false;
let dragStartX = 0, dragStartY = 0, panStartX = 0, panStartY = 0;
const DRAG_THRESHOLD = 6; // px of movement before a tap becomes a pan

canvas.addEventListener('pointerdown', (e) => {
  isPanning = true;
  dragMoved = false;
  dragStartX = e.clientX; dragStartY = e.clientY;
  panStartX = panX; panStartY = panY;
  canvas.setPointerCapture(e.pointerId);
  canvas.classList.add('dragging');
});

canvas.addEventListener('pointermove', (e) => {
  if (!isPanning) return;
  const dx = e.clientX - dragStartX;
  const dy = e.clientY - dragStartY;
  if (!dragMoved && Math.hypot(dx, dy) > DRAG_THRESHOLD) dragMoved = true;
  if (dragMoved) {
    panX = panStartX + dx;
    panY = panStartY + dy;
    clampPan();
    applyPan();
  }
});

canvas.addEventListener('pointerup', (e) => {
  isPanning = false;
  canvas.classList.remove('dragging');
  if (!dragMoved) {
    // it was a tap, not a pan -> place a waypoint
    if (path.length === 0) {
      pathOrigin = { x: botPos.x, y: botPos.y };  // snapshot bot position for THIS path only
    }
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const worldX = (mx - centerIdx * CELL_PX) / scalePxPerMM;
    const worldY = (centerIdx * CELL_PX - my) / scalePxPerMM;
    path.push({ x: worldX, y: worldY });
    drawPath();
  }
});

canvas.addEventListener('pointercancel', () => {
  isPanning = false;
  canvas.classList.remove('dragging');
});

function undoPoint() { path.pop(); drawPath(); }
function clearPath() { path = []; pathOrigin = { x: botPos.x, y: botPos.y }; drawPath(); }

// Jump the view back so the bot is centered in the fixed 400x400 box
function recenterOnBot() {
  const px = worldToPx(botPos.x, botPos.y);
  panX = VIEWPORT_PX / 2 - px.x;
  panY = VIEWPORT_PX / 2 - px.y;
  clampPan();
  applyPan();
}

function sendPath() {
  if (path.length === 0) return;
  const body = path.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(';');
  fetch('/path/upload', { method: 'POST', body: body })
    .then(() => fetch('/path/start'));
}

function resetPosition() {
  if (!confirm('Mark the bot\'s current spot/heading as the new (0,0) origin?')) return;
  fetch('/path/reset').then(() => { path = []; drawPath(); });
}

// ---------------- 360 scan ----------------
function startScan() { fetch('/scan/start').catch(() => {}); }
function clearScan() {
  if (!confirm('Clear the accumulated obstacle scan map?')) return;
  fetch('/scan/clear').then(fetchScan).catch(() => {});
}

function fetchScan() {
  fetch('/scan/points').then(r => r.json()).then(d => {
    scanPoints = d.points;
    if (d.scanning) {
      scanInfo.innerText = `Scanning... ${d.count} points so far`;
    } else if (d.count > 0) {
      scanInfo.innerText = `Last scan: ${d.count} points`;
    } else {
      scanInfo.innerText = 'No scan yet';
    }
    drawPath();
  }).catch(() => {});
}

function fetchPolylines() {
  fetch('/scan/polylines').then(r => r.json()).then(d => {
    cleanPolylines = d.polylines;
    drawPath();
  }).catch(() => {});
}

function worldToPx(x_mm, y_mm) {
  return {
    x: centerIdx * CELL_PX + x_mm * scalePxPerMM,
    y: centerIdx * CELL_PX - y_mm * scalePxPerMM
  };
}

function drawPath() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // grid lines
  ctx.strokeStyle = '#2d313b';
  ctx.lineWidth = 1;
  for (let i = 0; i <= GRID_N; i++) {
    ctx.beginPath();
    ctx.moveTo(i * CELL_PX, 0); ctx.lineTo(i * CELL_PX, GRID_N * CELL_PX);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, i * CELL_PX); ctx.lineTo(GRID_N * CELL_PX, i * CELL_PX);
    ctx.stroke();
  }

  // origin marker
  ctx.fillStyle = '#5a5f6b';
  ctx.beginPath();
  ctx.arc(centerIdx * CELL_PX, centerIdx * CELL_PX, 3, 0, 2 * Math.PI);
  ctx.fill();

  // drawn path (line + waypoint dots) - starts from the bot's current live
  // position, not the fixed world origin, so it stays accurate as the bot drives
  if (path.length > 0) {
    ctx.strokeStyle = '#3498db';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const startPx = worldToPx(pathOrigin.x, pathOrigin.y);
    ctx.moveTo(startPx.x, startPx.y);
    path.forEach(p => {
      const px = worldToPx(p.x, p.y);
      ctx.lineTo(px.x, px.y);
    });
    ctx.stroke();

    ctx.fillStyle = '#3498db';
    path.forEach(p => {
      const px = worldToPx(p.x, p.y);
      ctx.beginPath();
      ctx.arc(px.x, px.y, 5, 0, 2 * Math.PI);
      ctx.fill();
    });
  }

  // 360 scan obstacle points (raw point cloud - unchanged)
  ctx.fillStyle = '#f1c40f';
  scanPoints.forEach(p => {
    if (!p.valid) return;
    const px = worldToPx(p.x, p.y);
    ctx.beginPath();
    ctx.arc(px.x, px.y, 3, 0, 2 * Math.PI);
    ctx.fill();
  });

  // Clean, smoothed wall polylines (SOR -> ROR -> MLS -> DBSCAN -> NN
  // ordering -> Chaikin smoothing) - curved, drawn over the raw cloud
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  cleanPolylines.forEach(poly => {
    if (!poly.points || poly.points.length < 2) return;
    ctx.beginPath();
    const p0 = worldToPx(poly.points[0].x, poly.points[0].y);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < poly.points.length; i++) {
      const px = worldToPx(poly.points[i].x, poly.points[i].y);
      ctx.lineTo(px.x, px.y);
    }
    ctx.stroke();
  });

  // bot triangle (live position + heading)
  const px = worldToPx(botPos.x, botPos.y);
  ctx.save();
  ctx.translate(px.x, px.y);
  ctx.rotate(botPos.heading * Math.PI / 180);
  ctx.fillStyle = '#e74c3c';
  ctx.beginPath();
  ctx.moveTo(0, -11);
  ctx.lineTo(-7, 9);
  ctx.lineTo(7, 9);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

drawPath();

// ---------------- Tune panel ----------------
function populateTune(d) {
  document.getElementById('tBaseSpeed').value = d.baseSpeed;
  document.getElementById('tLeftScale').value = d.leftSpeedScale;
  document.getElementById('tRightScale').value = d.rightSpeedScale;
  document.getElementById('tLeftTicks').value = d.leftTicksPerRev;
  document.getElementById('tRightTicks').value = d.rightTicksPerRev;
  document.getElementById('tWheelDia').value = d.wheelDiameterMM;
  document.getElementById('tHeadingKP').value = d.headingKP;
  document.getElementById('tTurnMax').value = d.turnSpeedMax;
  document.getElementById('tTurnMin').value = d.turnSpeedMin;
  document.getElementById('tTurnSlow').value = d.turnSlowdownDeg;
  document.getElementById('tTurnTol').value = d.turnToleranceDeg;
  document.getElementById('tTurnLead').value = d.turnStopLeadDeg;
  document.getElementById('tPathTol').value = d.pathArriveTolMM;
  document.getElementById('tPathRev').checked = d.pathHeadingSign < 0;
  document.getElementById('tScanStep').value = d.scanStepDeg;
  document.getElementById('tScanSettle').value = d.scanSettleMs;
  document.getElementById('tInvLeft').checked = d.invertLeft;
  document.getElementById('tInvRight').checked = d.invertRight;
  document.getElementById('tInvTurn').checked = d.invertTurn;
  document.getElementById('tMinCell').value = d.minCellVoltage;
}

function fetchTune() {
  fetch('/tune/get').then(r => r.json()).then(populateTune);
}

function tuneQuery() {
  const g = id => document.getElementById(id).value;
  const c = id => document.getElementById(id).checked ? 1 : 0;
  const params = {
    baseSpeed: g('tBaseSpeed'), leftSpeedScale: g('tLeftScale'), rightSpeedScale: g('tRightScale'),
    leftTicksPerRev: g('tLeftTicks'), rightTicksPerRev: g('tRightTicks'), wheelDiameterMM: g('tWheelDia'),
    headingKP: g('tHeadingKP'),
    turnSpeedMax: g('tTurnMax'), turnSpeedMin: g('tTurnMin'), turnSlowdownDeg: g('tTurnSlow'),
    turnToleranceDeg: g('tTurnTol'), turnStopLeadDeg: g('tTurnLead'),
    pathArriveTolMM: g('tPathTol'), pathHeadingSign: document.getElementById('tPathRev').checked ? -1 : 1,
    scanStepDeg: g('tScanStep'), scanSettleMs: g('tScanSettle'),
    invertLeft: c('tInvLeft'), invertRight: c('tInvRight'), invertTurn: c('tInvTurn'),
    minCellVoltage: g('tMinCell')
  };
  return Object.entries(params).map(([k, v]) => `${k}=${encodeURIComponent(v)}`).join('&');
}

function applyTune() {
  fetch('/tune/set?' + tuneQuery()).then(r => r.json()).then(d => {
    populateTune(d);
    tuneMsg.innerText = 'Applied (not saved to EEPROM yet).';
  }).catch(() => { tuneMsg.innerText = 'Could not apply - bot may be busy.'; });
}

function saveTune() {
  fetch('/tune/save?' + tuneQuery()).then(r => r.json()).then(d => {
    populateTune(d);
    tuneMsg.innerText = 'Saved to EEPROM.';
  }).catch(() => { tuneMsg.innerText = 'Could not save - bot may be busy.'; });
}

function resetTune() {
  if (!confirm('Reset all tuning parameters to firmware defaults? (not yet saved to EEPROM)')) return;
  fetch('/tune/reset').then(r => r.json()).then(d => {
    populateTune(d);
    tuneMsg.innerText = 'Reset to firmware defaults (not saved to EEPROM yet).';
  }).catch(() => { tuneMsg.innerText = 'Could not reset - bot may be busy.'; });
}

fetchTune();

setInterval(() => {
  fetch('/status').then(r => r.json()).then(data => {
    distEl.innerText = data.distance_mm.toFixed(1) + " mm";
    headingEl.innerText = data.heading_deg.toFixed(1) + "\u00b0";
    stateEl.innerText = data.state;
    const busy = data.state !== "idle";
    dpadBtns.forEach(b => b.disabled = busy);
    turnBtns.forEach(b => b.disabled = busy);
    calibBtns.forEach(b => b.disabled = (data.state === "moving" || data.state === "turning"));
    pathBtns.forEach(b => b.disabled = busy);
    scanBtns.forEach(b => b.disabled = busy);
    tuneBtns.forEach(b => b.disabled = busy);

    botPos.x = data.pos_x_mm;
    botPos.y = data.pos_y_mm;
    botPos.heading = data.global_heading_deg;

    if (data.path_length > 0) {
      pathInfo.innerText = `Waypoint ${Math.min(data.path_index + 1, data.path_length)} / ${data.path_length}` +
        (data.path_following ? " (running)" : "");
    } else {
      pathInfo.innerText = "No path drawn yet";
    }

    drawPath();
  });
  refreshCalib();
  fetchScan();
  fetchPolylines();
}, 300);
</script>
</body>
</html>
)rawliteral";

// =========================================================================
// WEB SERVER HANDLERS
// =========================================================================

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Dashboard D-pad button was pressed (held) - start driving in the given direction.
// dir: fwd | bwd | left | right ("left"/"right" rotate in place, same convention
// as the ccw/cw Turns buttons: left == ccw, right == cw)
void handleMove() {
  if (turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  if (!server.hasArg("dir")) { server.send(400, "text/plain", "missing dir"); return; }

  String dir = server.arg("dir");
  if (dir == "fwd")        manualDir = DIR_FORWARD;
  else if (dir == "bwd")   manualDir = DIR_BACKWARD;
  else if (dir == "left")  manualDir = DIR_LEFT;
  else if (dir == "right") manualDir = DIR_RIGHT;
  else { server.send(400, "text/plain", "dir must be fwd/bwd/left/right"); return; }

  leftTicks = 0;
  rightTicks = 0;
  lastAvgDistMM = 0;
  headingDeg = 0;
  moving = true;
  lastPingMillis = millis();
  server.send(200, "text/plain", "moving");
}

// Dashboard calls this every ~150ms while the move button is held down,
// to prove the connection is still alive (see PING_TIMEOUT_MS above)
void handlePing() {
  lastPingMillis = millis();
  server.send(200, "text/plain", "pong");
}

void handleStop() {
  moving = false;
  manualDir = DIR_NONE;
  turning = false;
  stopMotors();
  server.send(200, "text/plain", "stopped");
}

// Starts a turn by a fixed number of degrees in a given direction
void handleTurn() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  if (!server.hasArg("dir") || !server.hasArg("deg")) {
    server.send(400, "text/plain", "missing dir/deg");
    return;
  }
  String dir = server.arg("dir");
  float deg = server.arg("deg").toFloat();
  if (dir != "cw" && dir != "ccw") { server.send(400, "text/plain", "dir must be cw or ccw"); return; }
  if (deg <= 0) { server.send(400, "text/plain", "deg must be positive"); return; }

  // If CW physically spins CCW on your bot, flip this line to (dir=="cw")?1:-1
  turnDirSign = (dir == "cw") ? -1 : 1;

  turnTargetDelta = deg;
  turnStartHeading = headingDeg;
  turning = true;
  server.send(200, "text/plain", "turning");
}

// Polled by the dashboard every 300ms to update distance/heading/state/position
void handleStatus() {
  float leftDist  = (fabs(leftTicks)  / leftTicksPerRev)  * wheelCircumferenceMM();
  float rightDist = (fabs(rightTicks) / rightTicksPerRev) * wheelCircumferenceMM();
  float avgDist = (leftDist + rightDist) / 2.0;

  String state = "idle";
  if (moving) state = "moving";
  else if (turning) state = "turning";
  else if (calibrating) state = "calibrating";
  else if (pathFollowing) state = "path";
  else if (scanning) state = "scanning";

  String json = "{\"distance_mm\":" + String(avgDist, 1) +
                ",\"heading_deg\":" + String(headingDeg, 1) +
                ",\"state\":\"" + state + "\"" +
                ",\"pos_x_mm\":" + String(botX, 1) +
                ",\"pos_y_mm\":" + String(botY, 1) +
                ",\"global_heading_deg\":" + String(globalHeading, 1) +
                ",\"path_following\":" + String(pathFollowing ? "true" : "false") +
                ",\"path_index\":" + String(pathIndex) +
                ",\"path_length\":" + String(pathLength) +
                ",\"cell1\":" + String(cell1Voltage, 2) +
                ",\"cell2\":" + String(cell2Voltage, 2) +
                "}";
  server.send(200, "application/json", json);
}

// ---------------- Calibration handlers ----------------
// These let you rotate the bot by hand through a known angle and compute
// a correction multiplier ("scale") so DMP-reported turns match reality.

void handleCalibStart() {
  if (moving || turning || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  calHeading = 0;
  calibrating = true;
  stopMotors();
  server.send(200, "text/plain", "calib started");
}

// Call this after rotating the bot by hand to roughly the given angle -
// computes how far off the raw DMP heading was and logs a scale sample
void handleCalibMark() {
  if (!calibrating) { server.send(409, "text/plain", "not calibrating"); return; }
  if (!server.hasArg("deg")) { server.send(400, "text/plain", "missing deg"); return; }
  float nominalDeg = server.arg("deg").toFloat();

  float raw = calHeading;
  if (fabs(raw) < 1.0) {
    server.send(400, "text/plain", "raw heading too small, rotate first");
    return;
  }
  float markScale = nominalDeg / fabs(raw);
  lastMarkScale = markScale;

  scaleSum += markScale; scaleCount++;

  String json = "{\"calibrating\":true"
                ",\"rawHeading\":" + String(raw, 1) +
                ",\"lastMarkScale\":" + String(markScale, 3) +
                ",\"scale\":" + String(scale, 3) + "}";
  server.send(200, "application/json", json);
}

void handleCalibStop() {
  calibrating = false;
  scaleSum = 0; scaleCount = 0;
  server.send(200, "text/plain", "calib stopped");
}

// Averages all "Mark" samples from this session and writes the result to EEPROM
void handleCalibSave() {
  if (scaleCount > 0) scale = scaleSum / scaleCount;

  CalibData d;
  d.magic = CAL_MAGIC;
  d.scale = scale;
  EEPROM.put(0, d);
  EEPROM.commit();

  scaleSum = 0; scaleCount = 0;
  calibrating = false;

  server.send(200, "text/plain", "saved");
}

void handleCalibStatus() {
  String json = "{\"calibrating\":" + String(calibrating ? "true" : "false") +
                ",\"rawHeading\":" + String(calHeading, 1) +
                ",\"lastMarkScale\":" + String(lastMarkScale, 3) +
                ",\"scale\":" + String(scale, 3) + "}";
  server.send(200, "application/json", json);
}

// Re-runs the gyro bias calibration - robot must be sitting still on a flat surface
void handleRebias() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  mpu.CalibrateGyro(6);
  haveLastYaw = false; // force a fresh reference next DMP packet
  server.send(200, "text/plain", "rebiased");
}

// ---------------- Tune handlers ----------------
// Every drive/turn/path/scan parameter (plus the three motor-direction
// booleans) lives in the "TUNABLE GLOBALS" block above. These handlers let
// the dashboard's Tune panel read them, apply new values in RAM, persist
// them to EEPROM, or reset back to the firmware DEFAULT_* values.

// Reads whichever tune args are present in the request and updates the
// matching global. Any arg not present is left unchanged, so partial
// updates are safe - but the dashboard always sends the full set.
void applyTuneArgs() {
  if (server.hasArg("baseSpeed"))        baseSpeed = server.arg("baseSpeed").toInt();
  if (server.hasArg("leftSpeedScale"))   leftSpeedScale = server.arg("leftSpeedScale").toFloat();
  if (server.hasArg("rightSpeedScale"))  rightSpeedScale = server.arg("rightSpeedScale").toFloat();
  if (server.hasArg("leftTicksPerRev"))  leftTicksPerRev = server.arg("leftTicksPerRev").toFloat();
  if (server.hasArg("rightTicksPerRev")) rightTicksPerRev = server.arg("rightTicksPerRev").toFloat();
  if (server.hasArg("wheelDiameterMM"))  wheelDiameterMM = server.arg("wheelDiameterMM").toFloat();
  if (server.hasArg("headingKP"))        headingKP = server.arg("headingKP").toFloat();
  if (server.hasArg("turnSpeedMax"))     turnSpeedMax = server.arg("turnSpeedMax").toInt();
  if (server.hasArg("turnSpeedMin"))     turnSpeedMin = server.arg("turnSpeedMin").toInt();
  if (server.hasArg("turnSlowdownDeg"))  turnSlowdownDeg = server.arg("turnSlowdownDeg").toFloat();
  if (server.hasArg("turnToleranceDeg")) turnToleranceDeg = server.arg("turnToleranceDeg").toFloat();
  if (server.hasArg("turnStopLeadDeg"))  turnStopLeadDeg = server.arg("turnStopLeadDeg").toFloat();
  if (server.hasArg("pathArriveTolMM"))  pathArriveTolMM = server.arg("pathArriveTolMM").toFloat();
  if (server.hasArg("pathHeadingSign"))  pathHeadingSign = (server.arg("pathHeadingSign").toInt() < 0) ? -1 : 1;
  if (server.hasArg("scanStepDeg"))      scanStepDeg = server.arg("scanStepDeg").toFloat();
  if (server.hasArg("scanSettleMs"))     scanSettleMs = server.arg("scanSettleMs").toInt();
  if (server.hasArg("invertLeft"))       invertLeft  = server.arg("invertLeft").toInt() != 0;
  if (server.hasArg("invertRight"))      invertRight = server.arg("invertRight").toInt() != 0;
  if (server.hasArg("invertTurn"))       invertTurn  = server.arg("invertTurn").toInt() != 0;
  if (server.hasArg("minCellVoltage"))   minCellVoltage = server.arg("minCellVoltage").toFloat();

  // Keep speeds/tolerances inside sane hardware ranges regardless of what
  // the browser sent.
  baseSpeed    = constrain(baseSpeed, 0, 255);
  turnSpeedMax = constrain(turnSpeedMax, 0, 255);
  turnSpeedMin = constrain(turnSpeedMin, 0, 255);
  if (wheelDiameterMM <= 0) wheelDiameterMM = DEFAULT_WHEEL_DIAMETER_MM;
  if (leftTicksPerRev <= 0) leftTicksPerRev = DEFAULT_LEFT_TICKS_PER_REV;
  if (rightTicksPerRev <= 0) rightTicksPerRev = DEFAULT_RIGHT_TICKS_PER_REV;
  if (minCellVoltage < 2.5 || minCellVoltage > 4.3) minCellVoltage = DEFAULT_MIN_CELL_VOLTAGE; // sane Li-ion/LiPo range
  if (scanStepDeg <= 0 || scanStepDeg > 90) scanStepDeg = DEFAULT_SCAN_STEP_DEG;
  if (scanSettleMs < 0) scanSettleMs = DEFAULT_SCAN_SETTLE_MS;
}

String buildTuneJson() {
  String json = "{";
  json += "\"baseSpeed\":" + String(baseSpeed) + ",";
  json += "\"leftSpeedScale\":" + String(leftSpeedScale, 3) + ",";
  json += "\"rightSpeedScale\":" + String(rightSpeedScale, 3) + ",";
  json += "\"leftTicksPerRev\":" + String(leftTicksPerRev, 1) + ",";
  json += "\"rightTicksPerRev\":" + String(rightTicksPerRev, 1) + ",";
  json += "\"wheelDiameterMM\":" + String(wheelDiameterMM, 2) + ",";
  json += "\"headingKP\":" + String(headingKP, 2) + ",";
  json += "\"turnSpeedMax\":" + String(turnSpeedMax) + ",";
  json += "\"turnSpeedMin\":" + String(turnSpeedMin) + ",";
  json += "\"turnSlowdownDeg\":" + String(turnSlowdownDeg, 1) + ",";
  json += "\"turnToleranceDeg\":" + String(turnToleranceDeg, 2) + ",";
  json += "\"turnStopLeadDeg\":" + String(turnStopLeadDeg, 1) + ",";
  json += "\"pathArriveTolMM\":" + String(pathArriveTolMM, 1) + ",";
  json += "\"pathHeadingSign\":" + String((int)pathHeadingSign) + ",";
  json += "\"scanStepDeg\":" + String(scanStepDeg, 1) + ",";
  json += "\"scanSettleMs\":" + String(scanSettleMs) + ",";
  json += "\"invertLeft\":" + String(invertLeft ? "true" : "false") + ",";
  json += "\"invertRight\":" + String(invertRight ? "true" : "false") + ",";
  json += "\"invertTurn\":" + String(invertTurn ? "true" : "false") + ",";
  json += "\"minCellVoltage\":" + String(minCellVoltage, 2);
  json += "}";
  return json;
}

void handleTuneGet() {
  server.send(200, "application/json", buildTuneJson());
}

// Applies new values in RAM only - lost on reboot unless /tune/save is
// called afterwards. Blocked while the bot is doing anything, since several
// of these parameters (speed, ticks/rev, inversions) are read mid-motion.
void handleTuneSet() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  applyTuneArgs();
  server.send(200, "application/json", buildTuneJson());
}

// Applies new values in RAM AND writes them to EEPROM so they survive a reboot.
void handleTuneSave() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  applyTuneArgs();

  TuneData td;
  td.magic = TUNE_MAGIC;
  td.baseSpeed = baseSpeed;
  td.leftSpeedScale = leftSpeedScale;
  td.rightSpeedScale = rightSpeedScale;
  td.leftTicksPerRev = leftTicksPerRev;
  td.rightTicksPerRev = rightTicksPerRev;
  td.wheelDiameterMM = wheelDiameterMM;
  td.headingKP = headingKP;
  td.turnSpeedMax = turnSpeedMax;
  td.turnSpeedMin = turnSpeedMin;
  td.turnSlowdownDeg = turnSlowdownDeg;
  td.turnToleranceDeg = turnToleranceDeg;
  td.turnStopLeadDeg = turnStopLeadDeg;
  td.pathArriveTolMM = pathArriveTolMM;
  td.pathHeadingSign = pathHeadingSign;
  td.invertLeft = invertLeft ? 1 : 0;
  td.invertRight = invertRight ? 1 : 0;
  td.invertTurn = invertTurn ? 1 : 0;
  td.minCellVoltage = minCellVoltage;
  td.scanStepDeg = scanStepDeg;
  td.scanSettleMs = scanSettleMs;
  EEPROM.put(TUNE_EEPROM_ADDR, td);
  EEPROM.commit();

  server.send(200, "application/json", buildTuneJson());
}

// Resets every tunable global back to the firmware DEFAULT_* values, in RAM
// only - call /tune/save afterwards if you also want that written to EEPROM.
void handleTuneReset() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }

  baseSpeed = DEFAULT_BASE_SPEED;
  leftSpeedScale = DEFAULT_LEFT_SPEED_SCALE;
  rightSpeedScale = DEFAULT_RIGHT_SPEED_SCALE;
  leftTicksPerRev = DEFAULT_LEFT_TICKS_PER_REV;
  rightTicksPerRev = DEFAULT_RIGHT_TICKS_PER_REV;
  wheelDiameterMM = DEFAULT_WHEEL_DIAMETER_MM;
  headingKP = DEFAULT_HEADING_KP;
  turnSpeedMax = DEFAULT_TURN_SPEED_MAX;
  turnSpeedMin = DEFAULT_TURN_SPEED_MIN;
  turnSlowdownDeg = DEFAULT_TURN_SLOWDOWN_DEG;
  turnToleranceDeg = DEFAULT_TURN_TOLERANCE_DEG;
  turnStopLeadDeg = DEFAULT_TURN_STOP_LEAD_DEG;
  pathArriveTolMM = DEFAULT_PATH_ARRIVE_TOL_MM;
  pathHeadingSign = DEFAULT_PATH_HEADING_SIGN;
  invertLeft = DEFAULT_INVERT_LEFT;
  invertRight = DEFAULT_INVERT_RIGHT;
  invertTurn = DEFAULT_INVERT_TURN;
  minCellVoltage = DEFAULT_MIN_CELL_VOLTAGE;
  scanStepDeg = DEFAULT_SCAN_STEP_DEG;
  scanSettleMs = DEFAULT_SCAN_SETTLE_MS;

  server.send(200, "application/json", buildTuneJson());
}

// ---------------- Path planning handlers ----------------

// Body format: "gx1,gy1;gx2,gy2;gx3,gy3" - integer GRID coordinates
// (each unit = GRID_CELL_MM), relative to wherever (0,0)/heading-0 currently is.
void handlePathUpload() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "missing body"); return; }

  String body = server.arg("plain");
  pathLength = 0;
  int start = 0;
  while (start < (int)body.length() && pathLength < MAX_PATH_POINTS) {
    int semi = body.indexOf(';', start);
    String pair = (semi == -1) ? body.substring(start) : body.substring(start, semi);
    int comma = pair.indexOf(',');
    if (comma > 0) {
      float px_mm = pair.substring(0, comma).toFloat();
      float py_mm = pair.substring(comma + 1).toFloat();
      pathPoints[pathLength].x = px_mm;
      pathPoints[pathLength].y = py_mm;
      pathLength++;

      // Live "Points : n / total" animation on the OLED as each point is
      // parsed. Since the whole path arrives in one HTTP body (not streamed
      // point-by-point), this plays back quickly right after parsing rather
      // than in true real time - it still gives the visual feedback from
      // the UI mockup, just compressed into a fraction of a second per point.
      if (oledOk) oledReceivingRoute(pathLength, -1); // total unknown until parse finishes
    }
    if (semi == -1) break;
    start = semi + 1;
  }
  pathIndex = 0;

  // Total path length, used later for the ROUTE SUMMARY screen. Starts
  // from (0,0) - wherever the bot's local origin currently is - through
  // every uploaded point in order.
  pathTotalLengthMM = 0;
  float prevX = 0, prevY = 0;
  for (int i = 0; i < pathLength; i++) {
    float dx = pathPoints[i].x - prevX;
    float dy = pathPoints[i].y - prevY;
    pathTotalLengthMM += sqrt(dx * dx + dy * dy);
    prevX = pathPoints[i].x;
    prevY = pathPoints[i].y;
  }

  if (oledOk) {
    oledRouteSummary(pathLength, pathTotalLengthMM / 1000.0);
    delay(1200); // brief pause so the summary is actually readable before Ready/dashboard resumes polling
  }

  server.send(200, "text/plain", "path uploaded, points=" + String(pathLength));
}

void handlePathStart() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  if (pathLength == 0) { server.send(400, "text/plain", "no path uploaded"); return; }
  pathIndex = 0;
  pathPaused = false;
  showRouteComplete = false;
  pathStartMillis = millis();
  pathFollowing = true;
  startPathWaypoint();
  server.send(200, "text/plain", "path following started");
}

void handlePathStop() {
  pathFollowing = false;
  pathPaused = false;
  pathSubState = PATH_IDLE;
  stopMotors();
  server.send(200, "text/plain", "path stopped");
}

// Freezes the bot mid-route without losing progress - pathIndex/pathSubState
// stay exactly where they were, so /path/resume continues the same
// turn-or-drive segment rather than restarting the current waypoint.
void handlePathPause() {
  if (!pathFollowing) { server.send(409, "text/plain", "not following a path"); return; }
  pathPaused = true;
  stopMotors();
  server.send(200, "text/plain", "paused");
}

void handlePathResume() {
  if (!pathFollowing) { server.send(409, "text/plain", "not following a path"); return; }
  pathPaused = false;
  server.send(200, "text/plain", "resumed");
}

// Marks the bot's current spot as the new (0,0) origin, facing heading 0.
// Also clears any uploaded path since its coordinates were relative to the old origin.
void handlePathReset() {
  if (moving || turning || calibrating || pathFollowing || scanning) { server.send(409, "text/plain", "busy"); return; }
  botX = 0; botY = 0; globalHeading = 0;
  pathLength = 0; pathIndex = 0;
  server.send(200, "text/plain", "position reset");
}

// ---------------- 360 scan handlers ----------------

void handleScanStart() {
  if (moving || turning || calibrating || pathFollowing || scanning) {
    server.send(409, "text/plain", "busy"); return;
  }
  if (!tofOk) { server.send(503, "text/plain", "tof sensor not detected"); return; }

  // Note: scanPointCount is intentionally NOT reset here - each new scan
  // appends onto whatever points already exist, so multiple scans build up
  // a combined obstacle map. Use /scan/clear to wipe it.
  scanStartHeading = headingDeg;   // reuse headingDeg as local progress tracker, same as turns
  scanRunStartIdx = scanPointCount;   // marks where this scan's points begin, for ICP registration later
  scanTraveled = 0;
  scanPointsThisRun = 0;
  scanSubState = SCAN_ROTATING;
  scanKicking = false;
  scanning = true;
  server.send(200, "text/plain", "scan started");
}

void handleScanStop() {
  scanning = false;
  stopMotors();
  server.send(200, "text/plain", "scan stopped");
}

// Wipes the accumulated obstacle map so the next scan starts from empty.
void handleScanClear() {
  if (scanning) { server.send(409, "text/plain", "busy"); return; }
  scanPointCount = 0;
  cleanPolylineCount = 0;
  server.send(200, "text/plain", "scan cleared");
}

// Returns whatever points have been collected so far, so the dashboard can
// draw progressively during the scan rather than waiting for it to finish.
// Returns the post-processed, smoothed polylines (one per detected
// cluster/wall), separate from the raw scan point cloud.
void handleScanPolylines() {
  String json = "{\"count\":" + String(cleanPolylineCount) + ",\"polylines\":[";
  for (int p = 0; p < cleanPolylineCount; p++) {
    if (p > 0) json += ",";
    json += "{\"points\":[";
    for (int i = 0; i < cleanPolylines[p].count; i++) {
      if (i > 0) json += ",";
      json += "{\"x\":" + String(cleanPolylines[p].pts[i].x, 1) +
              ",\"y\":" + String(cleanPolylines[p].pts[i].y, 1) + "}";
    }
    json += "]}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleScanPoints() {
  String json = "{\"scanning\":" + String(scanning ? "true" : "false") +
                ",\"count\":" + String(scanPointCount) + ",\"points\":[";
  for (int i = 0; i < scanPointCount; i++) {
    if (i > 0) json += ",";
    json += "{\"x\":" + String(scanPoints[i].x, 1) +
            ",\"y\":" + String(scanPoints[i].y, 1) +
            ",\"valid\":" + String(scanPoints[i].valid ? "true" : "false") + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// =========================================================================
// EEPROM
// =========================================================================

void loadCalibration() {
  CalibData d;
  EEPROM.get(0, d);
  if (d.magic == CAL_MAGIC) {
    scale = d.scale;
    Serial.print("Loaded calibration -> scale: "); Serial.println(scale);
  } else {
    Serial.println("No saved calibration, using default (1.0)");
  }
}

void loadTuneParams() {
  TuneData td;
  EEPROM.get(TUNE_EEPROM_ADDR, td);
  if (td.magic == TUNE_MAGIC) {
    baseSpeed = td.baseSpeed;
    leftSpeedScale = td.leftSpeedScale;
    rightSpeedScale = td.rightSpeedScale;
    leftTicksPerRev = td.leftTicksPerRev;
    rightTicksPerRev = td.rightTicksPerRev;
    wheelDiameterMM = td.wheelDiameterMM;
    headingKP = td.headingKP;
    turnSpeedMax = td.turnSpeedMax;
    turnSpeedMin = td.turnSpeedMin;
    turnSlowdownDeg = td.turnSlowdownDeg;
    turnToleranceDeg = td.turnToleranceDeg;
    turnStopLeadDeg = td.turnStopLeadDeg;
    pathArriveTolMM = td.pathArriveTolMM;
    pathHeadingSign = td.pathHeadingSign;
    invertLeft = td.invertLeft != 0;
    invertRight = td.invertRight != 0;
    invertTurn = td.invertTurn != 0;
    minCellVoltage = td.minCellVoltage;
    scanStepDeg = td.scanStepDeg;
    scanSettleMs = td.scanSettleMs;
    Serial.println("Loaded tuning parameters from EEPROM.");
  } else {
    Serial.println("No saved tuning parameters, using firmware defaults.");
  }
}

float readBatteryMV(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < BATTERY_NUM_SAMPLES; i++) {
    sum += analogReadMilliVolts(pin);
  }
  return sum / (float)BATTERY_NUM_SAMPLES;
}

void updateBatteryVoltages() {
  float packMV = readBatteryMV(PIN_PACK);
  float juncMV = readBatteryMV(PIN_JUNCTION);
  float packVoltage     = PACK_M * packMV + PACK_C;
  float junctionVoltage = JUNC_M * juncMV + JUNC_C;
  cell1Voltage = junctionVoltage;
  cell2Voltage = packVoltage - junctionVoltage;
}

void updateBuzzer() {
  // >0.2V guard skips a false trigger from a not-yet-settled reading at boot
  lowBatteryAlert = (cell1Voltage > 0.2 && cell1Voltage < minCellVoltage) ||
                    (cell2Voltage > 0.2 && cell2Voltage < minCellVoltage);

  if (!lowBatteryAlert) {
    if (buzzerOn) { digitalWrite(BUZZER_PIN, LOW); buzzerOn = false; }
    return;
  }

  unsigned long now = millis();
  if (buzzerOn && (now - buzzerLastToggleMillis >= BUZZER_BEEP_MS)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    buzzerLastToggleMillis = now;
  } else if (!buzzerOn && (now - buzzerLastToggleMillis >= BUZZER_GAP_MS)) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    buzzerLastToggleMillis = now;
  }
}

// =========================================================================
// SETUP
// =========================================================================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  updateBatteryVoltages();   // get a real reading before the first OLED draw

  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // ---- OLED first, so every later step in setup() can report through it ----
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) {
    Serial.println("OLED init failed - continuing without display.");
  } else {
    oledBootScreen();
    delay(1200);
  }

  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();
  loadTuneParams();

  mpu.initialize();
  bool mpuOk = mpu.testConnection();
  if (!mpuOk) {
    Serial.println("MPU6050 connection failed!");
  }

  // VL53L0X ToF sensor for the 360 scan feature - non-fatal if missing,
  // the rest of the bot still works, /scan/start will just refuse.
  tofOk = tof.begin();
  if (!tofOk) {
    Serial.println("VL53L0X not detected - 360 scan feature disabled.");
  } else {
    Serial.println("VL53L0X ready.");
  }

  // Motors and encoders have no automated self-test here (there's no sensor
  // feedback that isolates "driver/motor is present and working" at boot
  // without actually spinning them) - they're shown OK on the assumption
  // wiring matches the pin map. If a motor is actually dead, that'll surface
  // as a driving/torque problem later, not here.
  if (oledOk) oledSystemCheck(oledOk, mpuOk, true, true);
  delay(1200);

  if (!mpuOk) {
    // MPU6050 is required for heading/turns/path-following - without it the
    // rest of the firmware can't run safely, so stop here rather than limp
    // along with broken navigation.
    errorActive = true;
    errorMessage = "MPU6050 Timeout";
    errorCode = "E03";
    if (oledOk) oledErrorScreen(errorMessage, errorCode);
    Serial.println("Halting - power cycle the board after checking MPU6050 wiring.");
    while (true) { delay(1000); }
  }

  Serial.println("Initializing DMP...");
  uint8_t devStatus = mpu.dmpInitialize();

  // Default offsets - for best results, run the MPU6050 IMU_Zero example
  // once on the bench and paste your board's actual offsets here.
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    Serial.println("Running gyro/accel auto-calibration, keep still...");
    runCalibrationWithProgress();  // same total CalibrateAccel(6)/CalibrateGyro(6) work as before, just split into steps so the OLED can show progress
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready.");

    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
  } else {
    Serial.print("DMP init failed, code: ");
    Serial.println(devStatus);

    errorActive = true;
    errorMessage = "DMP Init Failed";
    errorCode = "E04";
    if (oledOk) oledErrorScreen(errorMessage, errorCode);
    while (true) { delay(1000); }
  }

  ledcAttach(AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(BIN2, PWM_FREQ, PWM_RES);
  stopMotors();

  pinMode(LEFT_C1, INPUT_PULLUP);
  pinMode(LEFT_C2, INPUT_PULLUP);
  pinMode(RIGHT_C1, INPUT);
  pinMode(RIGHT_C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_C1), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_C1), rightEncoderISR, RISING);

  // ---- Connect to your WiFi network instead of creating a hotspot ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  const unsigned long WIFI_TIMEOUT_MS = 20000;
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_TIMEOUT_MS) {
    delay(300);
    Serial.print(".");
    if (oledOk) {
      int percent = (int)(((millis() - wifiStart) * 100UL) / WIFI_TIMEOUT_MS);
      oledWifiConnecting(percent);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! Open this in your browser: http://");
    Serial.println(WiFi.localIP());

    String mdnsHost = "";
    if (MDNS.begin(HOSTNAME)) {
      Serial.print("Or try: http://");
      Serial.print(HOSTNAME);
      Serial.println(".local");
      mdnsHost = String(HOSTNAME) + ".local";
    }

    if (oledOk) {
      oledWifiConnected(WiFi.localIP().toString(), mdnsHost);
      delay(1800);
    }
  } else {
    Serial.println();
    Serial.println("WiFi connect failed - check WIFI_SSID/WIFI_PASS and try again.");
    // Not fatal - the bot can still be driven/tuned over USB serial and the
    // rest of setup() still runs, but the dashboard won't be reachable.
    if (oledOk) {
      oledErrorScreen("WiFi Connect Failed", "E01");
      delay(2500);
    }
  }

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/ping", handlePing);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);
  server.on("/turn", handleTurn);
  server.on("/calib/start", handleCalibStart);
  server.on("/calib/mark", handleCalibMark);
  server.on("/calib/stop", handleCalibStop);
  server.on("/calib/save", handleCalibSave);
  server.on("/calib/status", handleCalibStatus);
  server.on("/rebias", handleRebias);
  server.on("/tune/get", handleTuneGet);
  server.on("/tune/set", handleTuneSet);
  server.on("/tune/save", handleTuneSave);
  server.on("/tune/reset", handleTuneReset);
  server.on("/path/upload", HTTP_POST, handlePathUpload);
  server.on("/path/start", handlePathStart);
  server.on("/path/stop", handlePathStop);
  server.on("/path/pause", handlePathPause);
  server.on("/path/resume", handlePathResume);
  server.on("/path/reset", handlePathReset);
  server.on("/scan/start", handleScanStart);
  server.on("/scan/stop", handleScanStop);
  server.on("/scan/clear", handleScanClear);
  server.on("/scan/points", handleScanPoints);
  server.on("/scan/polylines", handleScanPolylines);
  server.begin();
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
  server.handleClient();

  if (millis() - lastBatteryUpdateMillis >= BATTERY_UPDATE_MS) {
    lastBatteryUpdateMillis = millis();
    updateBatteryVoltages();
  }
  updateBuzzer();
  // Safety: if the dashboard stops pinging (page closed, wifi dropped, etc.)
  // while driving forward, stop the motors automatically.
  if (moving && (millis() - lastPingMillis > PING_TIMEOUT_MS)) {
    moving = false;
    manualDir = DIR_NONE;
    stopMotors();
  }

  // Only touch the MPU6050 over I2C when it has told us (via interrupt)
  // that a fresh DMP packet is waiting.
  float rawDelta;
  bool gotYaw = false;
  if (mpuInterrupt) {
    mpuInterrupt = false;
    gotYaw = pollDMPYaw(rawDelta);
  }

  if (gotYaw) {
    // Global heading is tracked continuously, always, for the live dashboard.
    globalHeading += rawDelta * scale;

    if (moving || turning || pathFollowing || scanning) {
      headingDeg += rawDelta * scale;
    }
    if (calibrating) {
      calHeading += rawDelta;
    }
  }

  // Track live X/Y position only while actually translating in a straight
  // line (not while turning or scanning, since an in-place rotation spins
  // both wheels in opposite directions and would otherwise look like
  // spurious movement).
  bool manualTranslating = moving && (manualDir == DIR_FORWARD || manualDir == DIR_BACKWARD);
  if (manualTranslating || pathSubState == PATH_DRIVING) {
    updateOdometry();
  }

  if (moving) {
    switch (manualDir) {
      case DIR_FORWARD:  driveForwardWithCorrection();  break;
      case DIR_BACKWARD: driveBackwardWithCorrection(); break;
      case DIR_LEFT:     driveRotate(1);                break;  // ccw, same sense as Turns' "ccw"
      case DIR_RIGHT:    driveRotate(-1);               break;  // cw
      default: stopMotors(); break;
    }
  } else if (turning) {
    doTurnStep();
  } else if (pathFollowing && !pathPaused) {
    updatePathFollowing();
  } else if (scanning) {
    updateScan();
  }
  // While pathPaused is true, motors were already stopped in handlePathPause()
  // and updatePathFollowing() is simply not called - pathSubState/pathIndex
  // stay frozen until /path/resume clears the flag.

  if (oledOk) updateOled();
}

// =========================================================================
// MOTION / SENSOR HELPERS
// =========================================================================

// Reads one DMP FIFO packet and returns the wrapped yaw change (degrees)
// since the last successful read. Returns false if no packet was available.
bool pollDMPYaw(float &rawDeltaOut) {
  if (!dmpReady) return false;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yawDeg = ypr[0] * 180.0 / M_PI;

  if (!haveLastYaw) {
    lastYawDeg = yawDeg;
    haveLastYaw = true;
    rawDeltaOut = 0;
    return true;
  }

  float delta = yawDeg - lastYawDeg;
  // wrap to [-180, 180] so crossing the +-180 boundary doesn't spike
  while (delta > 180)  delta -= 360;
  while (delta < -180) delta += 360;

  lastYawDeg = yawDeg;
  rawDeltaOut = delta;
  return true;
}

// Updates botX/botY from how far the wheels have turned since the last call,
// projected along the current globalHeading. Call only while driving straight.
void updateOdometry() {
  float leftDist  = (fabs(leftTicks)  / leftTicksPerRev)  * wheelCircumferenceMM();
  float rightDist = (fabs(rightTicks) / rightTicksPerRev) * wheelCircumferenceMM();
  float avgDistMM = (leftDist + rightDist) / 2.0;

  float deltaMM = avgDistMM - lastAvgDistMM;
  lastAvgDistMM = avgDistMM;

  if (fabs(deltaMM) > 0.001) {
    float rad = globalHeading * PI / 180.0;
    botX += deltaMM * sin(rad);
    botY += deltaMM * cos(rad);
  }
}

// Drives both motors forward, steering by the accumulated heading error.
// Tune from the dashboard's Tune panel (Heading Kp, per-wheel speed scale,
// base speed) to improve straightness.
void driveForwardWithCorrection() {
  float correction = headingDeg * headingKP;

  int leftPWM  = (int)((baseSpeed * leftSpeedScale)  + correction);
  int rightPWM = (int)((baseSpeed * rightSpeedScale) - correction);

  leftPWM  = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  setMotor(true, leftPWM);
  setMotor(false, rightPWM);
}

// Mirror of driveForwardWithCorrection() for the D-pad's backward button.
// Same heading-hold logic, just running both wheels in reverse.
//
// NOTE: the correction sign here is flipped relative to
// driveForwardWithCorrection(). For a differential drive, angular velocity
// depends on (v_right - v_left), not on which way the robot is translating -
// so applying the exact same +correction/-correction pattern to left/right
// while both wheel velocities are negative (reverse) produces the OPPOSITE
// net turn compared to forward. That mismatch is what was turning small
// heading errors into a runaway (positive-feedback) drift instead of
// correcting them.
void driveBackwardWithCorrection() {
  float correction = headingDeg * headingKP;

  int leftPWM  = (int)((-baseSpeed * leftSpeedScale)  + correction);
  int rightPWM = (int)((-baseSpeed * rightSpeedScale) - correction);

  leftPWM  = constrain(leftPWM, -255, 0);
  rightPWM = constrain(rightPWM, -255, 0);

  setMotor(true, leftPWM);
  setMotor(false, rightPWM);
}

// Continuous in-place rotation for the D-pad's left/right buttons - spins at a
// fixed speed for as long as the button is held (released -> handleStop()).
// dirSign: +1 = ccw (left), -1 = cw (right) - matches handleTurn()'s convention.
void driveRotate(int dirSign) {
  int sign = dirSign * (invertTurn ? -1 : 1);
  setMotor(true,  sign * turnSpeedMax);
  setMotor(false, -sign * turnSpeedMax);
}

// Spins the bot toward the target turn angle, slowing down near the end.
// Tune Turn speed max/min, Slowdown start, and Coast lead from the Tune
// panel to improve turn accuracy.
void doTurnStep() {
  float turned = fabs(headingDeg - turnStartHeading);
  float remaining = turnTargetDelta - turned;

  // Stop early to account for mechanical coast after power is cut
  if (remaining <= turnStopLeadDeg) {
    stopMotors();
    turning = false;
    return;
  }

  int speed;
  if (remaining < turnSlowdownDeg) {
    speed = map((long)remaining, 0, (long)turnSlowdownDeg, turnSpeedMin, turnSpeedMax);
  } else {
    speed = turnSpeedMax;
  }
  speed = constrain(speed, turnSpeedMin, turnSpeedMax);

  int sign = turnDirSign * (invertTurn ? -1 : 1);
  setMotor(true,  sign * speed);
  setMotor(false, -sign * speed);
}

// Sets one motor's speed and direction. pwm range: -255..255, negative = reverse.
void setMotor(bool left, int pwm) {
  pwm = constrain(pwm, -255, 255);
  bool forward = pwm >= 0;
  int mag = abs(pwm);
  bool invert = left ? invertLeft : invertRight;
  if (invert) forward = !forward;

  int in1 = left ? AIN1 : BIN1;
  int in2 = left ? AIN2 : BIN2;

  if (forward) {
    ledcWrite(in1, mag);
    ledcWrite(in2, 0);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, mag);
  }
}

// Briefly brakes both motors (full duty both sides) before cutting power,
// which noticeably reduces coast/overshoot compared to just going to 0.
void stopMotors() {
  ledcWrite(AIN1, 255);
  ledcWrite(AIN2, 255);
  ledcWrite(BIN1, 255);
  ledcWrite(BIN2, 255);
  delay(100);
  ledcWrite(AIN1, 0);
  ledcWrite(AIN2, 0);
  ledcWrite(BIN1, 0);
  ledcWrite(BIN2, 0);
}

// =========================================================================
// PATH FOLLOWING
// =========================================================================

// Begins the TURN phase toward pathPoints[pathIndex]: figures out how far
// off the bot's current heading is from pointing at the target, and sets up
// turnStartHeading/turnTargetDelta/turnDirSign for updatePathFollowing() to
// drive toward zero, the same way a manual turn does.
void startPathWaypoint() {
  float dx = pathPoints[pathIndex].x - botX;
  float dy = pathPoints[pathIndex].y - botY;
  float distMM = sqrt(dx * dx + dy * dy);

  if (distMM < pathArriveTolMM) {
    advancePathWaypoint();  // already there, skip to the next point
    return;
  }

  // 0 = north(+Y), positive angle rotates toward +X - matches globalHeading's convention
  float targetHeading = atan2(dx, dy) * 180.0 / PI;
  float diff = targetHeading - globalHeading;
  while (diff > 180)  diff -= 360;
  while (diff < -180) diff += 360;
  diff *= pathHeadingSign;

  pathSegTargetDist = distMM;

  if (fabs(diff) < turnToleranceDeg) {
    beginPathDrive();  // already facing the right way, skip straight to driving
    return;
  }

  turnStartHeading = headingDeg;
  turnTargetDelta = fabs(diff);
  turnDirSign = (diff >= 0) ? 1 : -1;
  pathSubState = PATH_TURNING;
}

// Begins the DRIVE phase of the current waypoint: resets the local distance
// and heading-correction baselines, then hands off to driveForwardWithCorrection().
void beginPathDrive() {
  leftTicks = 0;
  rightTicks = 0;
  lastAvgDistMM = 0;
  headingDeg = 0;
  pathSubState = PATH_DRIVING;
}

void advancePathWaypoint() {
  pathIndex++;
  if (pathIndex >= pathLength) {
    stopMotors();

    // Latch stats for the ROUTE COMPLETE screen before pathFollowing flips
    // false, since the OLED's runtime state check happens after this.
    routeCompleteElapsedMs = millis() - pathStartMillis;
    routeCompletePoints = pathLength;
    showRouteComplete = true;
    routeCompleteShownAtMillis = millis();

    pathFollowing = false;
    pathPaused = false;
    pathSubState = PATH_IDLE;
  } else {
    startPathWaypoint();
  }
}

// Called every loop() while pathFollowing is true.
void updatePathFollowing() {
  if (pathSubState == PATH_TURNING) {
    float turned = fabs(headingDeg - turnStartHeading);
    float remaining = turnTargetDelta - turned;

    if (remaining <= turnStopLeadDeg) {
      stopMotors();
      beginPathDrive();
      return;
    }

    int speed;
    if (remaining < turnSlowdownDeg) {
      speed = map((long)remaining, 0, (long)turnSlowdownDeg, turnSpeedMin, turnSpeedMax);
    } else {
      speed = turnSpeedMax;
    }
    speed = constrain(speed, turnSpeedMin, turnSpeedMax);

    int sign = turnDirSign * (invertTurn ? -1 : 1);
    setMotor(true,  sign * speed);
    setMotor(false, -sign * speed);

  } else if (pathSubState == PATH_DRIVING) {
    float leftDist  = (fabs(leftTicks)  / leftTicksPerRev)  * wheelCircumferenceMM();
    float rightDist = (fabs(rightTicks) / rightTicksPerRev) * wheelCircumferenceMM();
    float avgDistMM = (leftDist + rightDist) / 2.0;

    float remaining = pathSegTargetDist - avgDistMM;
    if (remaining <= pathArriveTolMM) {
      stopMotors();
      advancePathWaypoint();
      return;
    }
    driveForwardWithCorrection();
  }
}

// =========================================================================
// 360 OBSTACLE SCAN
// =========================================================================

// Called every loop() while scanning is true. The bot rotates itself
// (no separate servo/mount motor) at turnSpeedMin, pausing every
// scanStepDeg degrees to let vibration settle (scanSettleMs) and then
// taking one VL53L0X reading. Each reading is converted to a world-frame
// (x,y) point using the bot's current position/heading, so results overlay
// correctly on the same canvas as the path-planning waypoints.
void updateScan() {
  switch (scanSubState) {

    case SCAN_ROTATING: {
      float turned = fabs(headingDeg - scanStartHeading);
      scanTraveled = turned;

      if (turned >= 360.0) {
        scanning = false;
        stopMotors();
        scanKicking = false;
        finalizeScanNode(scanRunStartIdx, scanPointCount);  // ICP registration + loop closure check
        return;  // full rotation done
      }

      // Rotate until we've covered the next scanStepDeg increment
      float nextTargetDeg = scanPointsThisRun * scanStepDeg;
      if (turned >= nextTargetDeg) {
        stopMotors();
        scanKicking = false;
        scanSubState = SCAN_SETTLING;
        scanSettleStartMillis = millis();
        return;
      }

      int sign = 1 * (invertTurn ? -1 : 1);  // fixed CCW sweep direction

      // Every step starts from a dead stop, so give it a brief full-power
      // kick to break static friction before dropping to the sustain speed -
      // a steady turnSpeedMin often isn't enough to get moving from rest
      // even though it's plenty once the wheels are already turning.
      if (!scanKicking) {
        scanKicking = true;
        scanKickStartMillis = millis();
      }

      int speed = (millis() - scanKickStartMillis < scanKickMs) ? turnSpeedMax : turnSpeedMin;
      setMotor(true,  sign * speed);
      setMotor(false, -sign * speed);
      break;
    }

    case SCAN_SETTLING:
      if (millis() - scanSettleStartMillis >= scanSettleMs) {
        scanSubState = SCAN_SAMPLING;
      }
      break;

    case SCAN_SAMPLING: {
      VL53L0X_RangingMeasurementData_t measure;
      tof.rangingTest(&measure, false);

      bool valid = (measure.RangeStatus != 4);  // 4 = out of range / no target
      float distMM = valid ? measure.RangeMilliMeter : 0;

      if (scanPointCount < MAX_SCAN_POINTS) {
        float rad = globalHeading * PI / 180.0;  // sensor faces "forward" = current heading
        scanPoints[scanPointCount].x = valid ? (botX + distMM * sin(rad)) : 0;
        scanPoints[scanPointCount].y = valid ? (botY + distMM * cos(rad)) : 0;
        scanPoints[scanPointCount].valid = valid;
        scanPointCount++;
        scanPointsThisRun++;
      } else {
        // Accumulated point buffer is full - stop rather than silently
        // dropping points from here on. Clear the map to keep scanning.
        scanning = false;
        stopMotors();
        return;
      }

      scanSubState = SCAN_ROTATING;
      break;
    }
  }
}

// =========================================================================
// OLED SCREENS
// =========================================================================
// Note on the checkmark glyphs from the UI mockup: Adafruit_GFX's built-in
// font doesn't include a check mark character, so "OK" / "FAIL" text is
// used instead - functionally the same information, just without the
// special glyph.

void oledBootScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(22, 4);
  display.println("WAYPOINT BOT");
  display.drawLine(0, 15, 127, 15, SSD1306_WHITE);
  display.setCursor(0, 28);
  display.println("Initializing...");
  display.setCursor(0, 48);
  display.println("Please Wait");
  display.display();
}

void oledSystemCheck(bool oledPass, bool mpuPass, bool motorsPass, bool encodersPass) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(16, 0);
  display.println("SYSTEM CHECK");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("OLED      ");
  display.println(oledPass ? "OK" : "FAIL");

  display.setCursor(0, 28);
  display.print("MPU6050   ");
  display.println(mpuPass ? "OK" : "FAIL");

  display.setCursor(0, 40);
  display.print("Motors    ");
  display.println(motorsPass ? "OK" : "FAIL");

  display.setCursor(0, 52);
  display.print("Encoders  ");
  display.println(encodersPass ? "OK" : "FAIL");

  display.display();
}

void oledWifiConnecting(int percent) {
  percent = constrain(percent, 0, 100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(50, 0);
  display.println("WIFI");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("Connecting...");
  display.setCursor(0, 28);
  display.println(WIFI_SSID);

  display.drawRect(0, 44, 122, 12, SSD1306_WHITE);
  int fillW = map(percent, 0, 100, 0, 118);
  display.fillRect(2, 46, fillW, 8, SSD1306_WHITE);

  display.display();
}

void oledWifiConnected(String ip, String mdnsHost) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(50, 0);
  display.println("WIFI");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("Connected OK");
  display.setCursor(0, 30);
  display.println(ip);
  display.setCursor(0, 44);
  if (mdnsHost.length() > 0) display.println(mdnsHost);
  display.display();
}

// Called in place of the old single blocking CalibrateAccel(6)/CalibrateGyro(6)
// calls. Splits the same total calibration work into 1-loop increments so
// the OLED can show real (not simulated) progress between calls.
void runCalibrationWithProgress() {
  const int steps = 6;
  for (int i = 1; i <= steps; i++) {
    mpu.CalibrateAccel(1);
    if (oledOk) oledCalibrationScreen(map(i, 0, steps, 0, 50));
  }
  for (int i = 1; i <= steps; i++) {
    mpu.CalibrateGyro(1);
    if (oledOk) oledCalibrationScreen(map(i, 0, steps, 50, 100));
  }
}

void oledCalibrationScreen(int percent) {
  percent = constrain(percent, 0, 100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(12, 0);
  display.println("CALIBRATION");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("Keep Robot Still");

  display.drawRect(0, 30, 122, 12, SSD1306_WHITE);
  int fillW = map(percent, 0, 100, 0, 118);
  display.fillRect(2, 32, fillW, 8, SSD1306_WHITE);

  display.setCursor(0, 50);
  display.print("Progress : ");
  display.print(percent);
  display.println("%");

  display.display();
}

void oledErrorScreen(String msg, String code) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(40, 0);
  display.println("ERROR");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(msg);
  display.setCursor(0, 34);
  display.println("Restart Required");
  display.setCursor(0, 48);
  display.print("Code : ");
  display.println(code);
  display.display();
}

// Plays a quick 0->pathLength animation right after parsing an uploaded
// path. See the comment in handlePathUpload() for why this isn't truly
// real-time (the whole path arrives in a single HTTP body, not point by
// point), so "total" here is passed as -1 mid-parse and only becomes
// meaningful once oledRouteSummary() is shown right after.
void oledReceivingRoute(int pointsSoFar, int total) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(24, 0);
  display.println("RECEIVING");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("Downloading Route");

  int totalForBar = (total > 0) ? total : max(pointsSoFar, 1);
  int percent = map(pointsSoFar, 0, totalForBar, 0, 100);
  display.drawRect(0, 30, 122, 12, SSD1306_WHITE);
  int fillW = map(percent, 0, 100, 0, 118);
  display.fillRect(2, 32, fillW, 8, SSD1306_WHITE);

  display.setCursor(0, 50);
  display.print("Points : ");
  display.print(pointsSoFar);
  if (total > 0) { display.print(" / "); display.print(total); }
  display.display();
}

void oledRouteSummary(int points, float lengthMeters) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(40, 0);
  display.println("ROUTE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 18);
  display.print("Points : ");
  display.println(points);
  display.setCursor(0, 32);
  display.print("Length : ");
  display.print(lengthMeters, 2);
  display.println(" m");
  display.setCursor(0, 46);
  display.println("Ready to Start");
  display.display();
}

void oledReadyScreen() {
  display.clearDisplay();
  display.setTextSize(1);

  // Battery row: C1 x.xxV | C2 x.xxV
  display.setCursor(0, 0);
  display.print("C1 ");
  display.print(cell1Voltage, 2);
  display.print("V");
  display.setCursor(60, 0);
  display.print("|");
  display.setCursor(70, 0);
  display.print("C2 ");
  display.print(cell2Voltage, 2);
  display.print("V");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(46, 14);
  display.println("READY");

  display.setCursor(0, 26);
  display.println("Waiting for Route");

  display.setCursor(0, 38);
  display.print("Heading : ");
  float headingWrapped = fmod(globalHeading, 360.0);
  if (headingWrapped < 0) headingWrapped += 360.0;
  display.print((int)headingWrapped);
  display.println(" deg");

  display.setCursor(0, 50);
  display.print("Encoders : ");
  display.println("OK");

  display.display();
}

void oledMovingToWaypoint() {
  float dx = pathPoints[pathIndex].x - botX;
  float dy = pathPoints[pathIndex].y - botY;
  float distMM = sqrt(dx * dx + dy * dy);

  float headingWrapped = fmod(globalHeading, 360.0);
  if (headingWrapped < 0) headingWrapped += 360.0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(28, 0);
  display.println("WAYPOINT");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("Point : ");
  display.print(pathIndex + 1);
  display.print(" / ");
  display.println(pathLength);

  display.setCursor(0, 28);
  display.print("Target: (");
  display.print((int)(pathPoints[pathIndex].x / GRID_CELL_MM));
  display.print(",");
  display.print((int)(pathPoints[pathIndex].y / GRID_CELL_MM));
  display.println(")");

  display.setCursor(0, 40);
  display.print("Head  : ");
  display.print((int)headingWrapped);
  display.println(" deg");

  display.setCursor(0, 52);
  display.print("Dist  : ");
  display.print((int)(distMM / 10.0));
  display.println(" cm");

  display.display();
}

// direction label follows turnDirSign (post pathHeadingSign) the same way
// the motor sign logic does elsewhere in this file - if the label comes out
// backwards versus the bot's actual physical rotation, that's the same kind
// of sign convention mismatch documented for invertTurn/pathHeadingSign in
// the Tune panel, not a bug in this display function itself.
void oledTurning() {
  bool isRight = (turnDirSign * (invertTurn ? -1 : 1)) < 0;
  float targetHeadingAbs = turnStartHeading + turnDirSign * turnTargetDelta;
  float turned = fabs(headingDeg - turnStartHeading);
  float remaining = turnTargetDelta - turned;
  if (remaining < 0) remaining = 0;

  float currentWrapped = fmod(globalHeading, 360.0);
  if (currentWrapped < 0) currentWrapped += 360.0;
  float targetWrapped = fmod(targetHeadingAbs, 360.0);
  if (targetWrapped < 0) targetWrapped += 360.0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(30, 0);
  display.println("TURNING");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("Direction : ");
  display.println(isRight ? "RIGHT" : "LEFT");

  display.setCursor(0, 28);
  display.print("Target : ");
  display.print((int)targetWrapped);
  display.println(" deg");

  display.setCursor(0, 40);
  display.print("Current: ");
  display.print((int)currentWrapped);
  display.println(" deg");

  display.setCursor(0, 52);
  display.print("Error  : ");
  display.print((int)remaining);
  display.println(" deg");

  display.display();
}

void oledPaused() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(40, 0);
  display.println("PAUSED");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Route Paused");
  display.setCursor(0, 34);
  display.print("Point : ");
  display.print(pathIndex + 1);
  display.print(" / ");
  display.println(pathLength);
  display.setCursor(0, 48);
  display.println("Awaiting Resume");
  display.display();
}

void oledRouteComplete(int points, unsigned long elapsedMs, float lengthMeters) {
  unsigned long totalSec = elapsedMs / 1000;
  int mins = totalSec / 60;
  int secs = totalSec % 60;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(8, 0);
  display.println("ROUTE COMPLETE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 18);
  display.print("Points : ");
  display.print(points);
  display.print(" / ");
  display.println(points);

  display.setCursor(0, 32);
  display.print("Time   : ");
  if (mins < 10) display.print("0");
  display.print(mins);
  display.print(":");
  if (secs < 10) display.print("0");
  display.println(secs);

  display.setCursor(0, 46);
  display.print("Distance: ");
  display.print(lengthMeters, 2);
  display.println(" m");

  display.display();
}

// Shown on the OLED while a 360 scan is in progress - mirrors the style of
// oledMovingToWaypoint()/oledTurning() but reports scan progress instead.
void oledScanning() {
  int approxPoints = (int)(360.0 / scanStepDeg);
  if (approxPoints < 1) approxPoints = 1;

  float headingWrapped = fmod(globalHeading, 360.0);
  if (headingWrapped < 0) headingWrapped += 360.0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(30, 0);
  display.println("SCANNING");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("Progress: ");
  display.print((int)scanTraveled);
  display.println(" / 360 deg");

  display.drawRect(0, 26, 122, 10, SSD1306_WHITE);
  int fillW = map(constrain((int)scanTraveled, 0, 360), 0, 360, 0, 118);
  display.fillRect(2, 28, fillW, 6, SSD1306_WHITE);

  display.setCursor(0, 42);
  display.print("Points  : ");
  display.print(scanPointCount);
  display.print(" / ~");
  display.println(approxPoints);

  display.setCursor(0, 54);
  display.print("Heading : ");
  display.print((int)headingWrapped);
  display.println(" deg");

  display.display();
}

// Picks which screen to show based on current robot state. Called once per
// OLED_UPDATE_MS from loop(). A persistent errorActive halts the firmware
// in setup() before this is ever reached, so it's not checked here again.
void updateOled() {
  if (millis() - lastOledUpdateMillis < OLED_UPDATE_MS) return;
  lastOledUpdateMillis = millis();

  if (showRouteComplete) {
    if (millis() - routeCompleteShownAtMillis < ROUTE_COMPLETE_DISPLAY_MS) {
      oledRouteComplete(routeCompletePoints, routeCompleteElapsedMs, pathTotalLengthMM / 1000.0);
      return;
    } else {
      showRouteComplete = false; // fall through to Ready screen below
    }
  }

  if (scanning) {
    oledScanning();
  } else if (pathFollowing && pathPaused) {
    oledPaused();
  } else if (pathFollowing && pathSubState == PATH_TURNING) {
    oledTurning();
  } else if (pathFollowing && pathSubState == PATH_DRIVING) {
    oledMovingToWaypoint();
  } else {
    // Covers idle, manual move/turn/calibration sessions, and the moment
    // right after a path finishes (once showRouteComplete's window expires).
    oledReadyScreen();
  }
}
