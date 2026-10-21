#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// =====================================================
// MIMO v0.1.0 — FLUID + EXPRESSIVE FACE + MOTION WAKE
// ESP32-C3 SuperMini / MPU6500 / SH1106 128x64
//
// FLUID: unchanged from v7 (your working version)
// DEEP SLEEP / MOTION WAKE / SWITCH: unchanged
//
// FACE (new expression system):
// - Springy eyes (squash & stretch with movement, overshoot
//   when they pop open)
// - WAKING   : eyes pop open from a flat line + blink
// - IDLE     : looks around, blinks, plus random extras:
//              wink, heart eyes, quick side glance, double blink
// - SURPRISED: hard shake -> wide eyes, "o" mouth (as before)
// - DIZZY    : shaking for a while -> spiral eyes, wobbly mouth
// - SQUINT   : gentle swaying -> happy ^ ^ eyes, blush, grin
// - WORRIED  : device upside down -> sad eyebrows, frown
// - SLEEPY   : ~2.2 s without motion -> eyelids droop, yawn,
//              floating "z"s, then deep sleep at 5 s
// =====================================================

// ---------------- PINS ----------------

#define SDA_PIN       8
#define SCL_PIN       9
#define SWITCH_PIN    10
#define MPU_INT_PIN   3

// ---------------- DISPLAY ----------------

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

// ---------------- MPU6500 ----------------

uint8_t mpuAddr = 0x68;
bool mpuOK = false;

float ax = 0, ay = 0, az = 1;
float gx = 0, gy = 0, gz = 0;

float fx = 0, fy = 0, fz = 1;
bool gravityReady = false;

// Dedicated liquid gravity.
// horizontal = +AY, vertical = -AX.
float liquidGravityX = 0;
float liquidGravityY = 0;
bool liquidGravityReady = false;

// ---------------- TIMING ----------------

unsigned long lastSensor = 0;
unsigned long lastFrame = 0;
unsigned long lastSplash = 0;
unsigned long lastActivity = 0;

const unsigned long SENSOR_INTERVAL = 10;
const unsigned long FRAME_INTERVAL = 25;
const unsigned long SLEEP_TIMEOUT = 10000;

// ---------------- SWITCH ----------------

bool switchLast = HIGH;
bool liquidMode = true;

// =====================================================
// SHARED MOTION DETECTION
// =====================================================

float motionPrevAx = 0;
float motionPrevAy = 0;
float motionPrevAz = 1;
bool motionReferenceReady = false;

float shakeEnergy = 0;

unsigned long lastShakeTime = 0;
unsigned long lastActivityEvent = 0;

const float ACTIVITY_GYRO_THRESHOLD = 100.0f;
const float ACTIVITY_ACCEL_THRESHOLD = 0.20f;

const float SHAKE_GYRO_THRESHOLD = 135.0f;
const float SHAKE_ACCEL_THRESHOLD = 0.38f;

const unsigned long SHAKE_COOLDOWN = 300;

// =====================================================
// LIQUID SIMULATION
// =====================================================
//
// Units: positions are in 1/256 pixel. Velocities are in
// 1/256 pixel per simulation substep.

#define LIQ_COLS 60
#define LIQ_ROWS 8
const int N = LIQ_COLS * LIQ_ROWS;        // 280 particles

const float SIM_DT = 1.0f / 60.0f;        // fixed physics step
const int   MAX_SUBSTEPS = 3;             // cap work per frame
const int   SOLVER_ITERS = 3;             // more = stiffer liquid

const int32_t REST_DIST = 700;            // ~2.7 px particle spacing
const int32_t REST_DIST2 = REST_DIST * REST_DIST;

const int32_t MAX_SPEED = 600;            // < REST_DIST: no tunnelling
const int   DAMPING_SHIFT = 6;            // v -= v/64 per substep

// Orientation fix. Applied on top of the base mapping, in this order:
// swap X<->Y first, then invert. Change ONE flag at a time.
const bool LIQ_SWAP_XY = false;
const bool LIQ_INVERT_X = true;           // left/right was reversed
const bool LIQ_INVERT_Y = false;

const float GRAVITY_CLAMP = 100.0f;       // ~3 g
const float DESK_BIAS = 10.0f;            // soft "down" when device is flat

// Container limits (particle centre, 3x3 drawn around it).
const int32_t MIN_X = 1 * 256;
const int32_t MAX_X = 126 * 256;
const int32_t MIN_Y = 1 * 256;
const int32_t MAX_Y = 62 * 256;

// Spatial hash grid: 4 px cells.
#define GRID_W 32
#define GRID_H 16

int32_t px_[N], py_[N];     // position
int32_t ox_[N], oy_[N];     // position before integration
int32_t vx_[N], vy_[N];     // velocity

int16_t gridHead[GRID_W * GRID_H];
int16_t nextInCell[N];

float simAccumulator = 0;

float previousAx = 0;
float previousAy = 0;
float previousAz = 1;

float previousShakeEnergy = 0;

// =====================================================
// FACE: STATE, SPRINGS, TUNING
// =====================================================

enum FaceState {
  FACE_IDLE,
  FACE_SURPRISED,
  FACE_DIZZY,
  FACE_WORRIED,
  FACE_SLEEPY,
  FACE_WAKING,
  FACE_SQUINT
};

FaceState faceState = FACE_WAKING;

// --- tuning knobs ---
const unsigned long SLEEPY_START_MS = 2200;  // idle time when droop starts
const unsigned long WAKE_DURATION   = 1500;  // wake-up animation length

const float DIZZY_ENERGY = 2.1f;   // shakeEnergy that counts as "shaking hard"
const float DIZZY_TIME   = 1.1f;   // seconds of hard shaking -> dizzy
const unsigned long DIZZY_DURATION = 5000;

const float SWAY_MIN = 18.0f;      // deg/s: gentle motion band for happy squint
const float SWAY_MAX = 95.0f;

// If the worried face triggers when the device is the RIGHT way up
// (and not when it is upside down), flip this to true.
const bool FACE_UPSIDE_INVERT = false;

// true = gentle swaying keeps it awake (resets the 5 s timer).
const bool SWAY_KEEPS_AWAKE = false;

// --- spring-animated eye parameters ---
struct Spring {
  float v;
  float vel;
};

Spring sW     = {45.0f, 0};   // eye width
Spring sH     = {5.0f,  0};   // eye height
Spring sLid   = {0.0f,  0};   // top eyelid, fraction of height
Spring sSlant = {0.0f,  0};   // sad/worried corner cut, pixels
Spring sRad   = {3.0f,  0};   // corner radius

void springStep(Spring &s, float target, float step) {
  const float K = 220.0f;   // stiffness
  const float C = 16.0f;    // damping (lower = more bounce)

  s.vel += (target - s.v) * K * step;
  s.vel -= s.vel * C * step;
  s.v   += s.vel * step;
}

// --- blink / gaze (as before) ---
float buddyBob = 0;

unsigned long nextBlink = 0;
unsigned long blinkStart = 0;
bool blinking = false;
int extraBlinks = 0;

unsigned long nextLookTime = 0;

bool buddyReacting = false;
unsigned long reactionUntil = 0;

const unsigned long REACTION_DURATION = 850;

float eyeOffsetX = 0;
float eyeOffsetY = 0;
float targetEyeOffsetX = 0;
float targetEyeOffsetY = 0;

float idleGazeX = 0;
float idleGazeY = 0;

// --- new face state ---
unsigned long wakeStart = 0;
bool wakeBlinkDone = false;

float sway = 0;
float squintTimer = 0;
unsigned long squintHold = 0;

float dizzyMeter = 0;
bool dizzy = false;
unsigned long dizzyUntil = 0;

bool upsideDown = false;
float udTimer = 0;

float sleepProgress = 0;   // 0..1 during the pre-sleep droop
float yawnOpen = 0;        // 0..1

unsigned long nextIdleEvent = 0;
unsigned long winkUntil = 0;
unsigned long heartUntil = 0;
unsigned long glanceUntil = 0;
float glanceX = 0;
float glanceY = 0;

// =====================================================
// HELPERS
// =====================================================

int clampInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((uint8_t)mpuAddr, len) != len)
    return false;

  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.read();

  return true;
}

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  uint8_t value = 0xFF;
  readRegs(reg, &value, 1);
  return value;
}

// =====================================================
// MPU6500 NORMAL INITIALIZATION
// =====================================================

bool initMPU() {
  const uint8_t addresses[] = {0x68, 0x69};

  for (int i = 0; i < 2; i++) {
    mpuAddr = addresses[i];

    uint8_t id = 0;

    if (!readRegs(0x75, &id, 1) ||
        id == 0x00 || id == 0xFF)
      continue;

    writeReg(0x6B, 0x80);
    delay(100);

    writeReg(0x6B, 0x00);
    delay(100);

    writeReg(0x1C, 0x00);
    writeReg(0x1B, 0x00);
    writeReg(0x1A, 0x03);
    writeReg(0x38, 0x00);
    writeReg(0x37, 0x00);

    Serial.printf(
      "MPU address: 0x%02X, WHO_AM_I: 0x%02X\n",
      mpuAddr, id
    );

    gravityReady = false;
    liquidGravityReady = false;
    motionReferenceReady = false;

    previousAx = 0;
    previousAy = 0;
    previousAz = 1;

    return true;
  }

  return false;
}

bool readMPU() {
  uint8_t d[14];

  if (!readRegs(0x3B, d, 14))
    return false;

  int16_t rx = (int16_t)((d[0] << 8) | d[1]);
  int16_t ry = (int16_t)((d[2] << 8) | d[3]);
  int16_t rz = (int16_t)((d[4] << 8) | d[5]);

  int16_t rGX = (int16_t)((d[8] << 8) | d[9]);
  int16_t rGY = (int16_t)((d[10] << 8) | d[11]);
  int16_t rGZ = (int16_t)((d[12] << 8) | d[13]);

  ax = rx / 16384.0f;
  ay = ry / 16384.0f;
  az = rz / 16384.0f;

  gx = rGX / 131.0f;
  gy = rGY / 131.0f;
  gz = rGZ / 131.0f;

  if (!gravityReady) {
    fx = ax;
    fy = ay;
    fz = az;

    previousAx = ax;
    previousAy = ay;
    previousAz = az;

    gravityReady = true;
  }

  // Filter gravity for face behavior.
  float alpha = 1.0f - expf(-0.01f / 0.22f);
  float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  if (magnitude > 0.78f && magnitude < 1.22f) {
    fx += (ax - fx) * alpha;
    fy += (ay - fy) * alpha;
    fz += (az - fz) * alpha;
  }

  // Liquid orientation mapping:
  // liquidGravityX = AY(g) * 32
  // liquidGravityY = -AX(g) * 32
  float targetLiquidX = ay * 32.0f;
  float targetLiquidY = -ax * 32.0f;

  if (!liquidGravityReady) {
    liquidGravityX = targetLiquidX;
    liquidGravityY = targetLiquidY;
    liquidGravityReady = true;
  } else {
    liquidGravityX +=
      (targetLiquidX - liquidGravityX) * 0.25f;

    liquidGravityY +=
      (targetLiquidY - liquidGravityY) * 0.25f;
  }

  return true;
}

// =====================================================
// SHARED MOTION DETECTION
// =====================================================

void processMotion(unsigned long now) {
  if (!mpuOK)
    return;

  float gyroMagnitude = sqrtf(
    gx * gx + gy * gy + gz * gz
  );

  if (!motionReferenceReady) {
    motionPrevAx = ax;
    motionPrevAy = ay;
    motionPrevAz = az;
    motionReferenceReady = true;
    return;
  }

  float deltaA =
      fabsf(ax - motionPrevAx)
    + fabsf(ay - motionPrevAy)
    + fabsf(az - motionPrevAz);

  motionPrevAx = ax;
  motionPrevAy = ay;
  motionPrevAz = az;

  shakeEnergy *= 0.88f;
  shakeEnergy += deltaA;

  bool significantMotion =
      gyroMagnitude > ACTIVITY_GYRO_THRESHOLD
      || deltaA > ACTIVITY_ACCEL_THRESHOLD;

  bool strongShake =
      gyroMagnitude > SHAKE_GYRO_THRESHOLD
      || deltaA > SHAKE_ACCEL_THRESHOLD;

  if (significantMotion) {
    lastActivity = now;
    lastActivityEvent = now;
  }

  if (!liquidMode && strongShake &&
      now - lastShakeTime > SHAKE_COOLDOWN) {

    lastShakeTime = now;
    buddyReacting = true;
    reactionUntil = now + REACTION_DURATION;

    idleGazeX = 0;
    idleGazeY = 0;

    lastActivity = now;
  }
}

// =====================================================
// MOTION WAKE CONFIGURATION
// PRESERVED FROM YOUR WORKING SKETCH
// =====================================================

void configureMotionWakeup() {
  writeReg(0x6B, 0x00);
  delay(100);

  writeReg(0x6C, 0x07);
  writeReg(0x1D, 0x01);
  writeReg(0x1F, 80);
  writeReg(0x1E, 0x05);
  writeReg(0x37, 0x20);
  writeReg(0x69, 0xC0);
  writeReg(0x38, 0x40);
  writeReg(0x6B, 0x20);

  delay(100);

  uint8_t status = 0;
  readRegs(0x3A, &status, 1);
}

// =====================================================
// DEEP SLEEP / MOTION WAKE
// PRESERVED FROM YOUR WORKING SKETCH
// =====================================================

void enterDeepSleep() {
  if (!mpuOK)
    return;

  Serial.println();
  Serial.println("Preparing for deep sleep...");
  Serial.flush();

  oled.setPowerSave(1);

  configureMotionWakeup();

  pinMode(MPU_INT_PIN, INPUT);

  uint8_t status = 0;

  for (int i = 0; i < 5; i++) {
    readRegs(0x3A, &status, 1);
    delay(20);
  }

  unsigned long start = millis();

  while (digitalRead(MPU_INT_PIN) == HIGH &&
         millis() - start < 500) {
    readRegs(0x3A, &status, 1);
    delay(10);
  }

  int intLevel = digitalRead(MPU_INT_PIN);

  Serial.printf("MPU INT before sleep: %d\n", intLevel);
  Serial.printf("MPU INT_STATUS: 0x%02X\n", status);
  Serial.flush();

  if (intLevel == HIGH) {
    Serial.println("INT remains HIGH; cancelling sleep.");
    Serial.flush();

    oled.setPowerSave(0);

    mpuOK = initMPU();

    lastActivity = millis();
    lastSensor = millis();
    lastFrame = millis();

    return;
  }

  esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(
    1ULL << MPU_INT_PIN,
    ESP_GPIO_WAKEUP_GPIO_HIGH
  );

  if (err != ESP_OK) {
    Serial.printf("GPIO wake configuration failed: %d\n", err);
    Serial.flush();

    oled.setPowerSave(0);

    mpuOK = initMPU();

    lastActivity = millis();
    lastSensor = millis();
    lastFrame = millis();

    return;
  }

  Serial.println("Entering deep sleep now.");
  Serial.flush();

  esp_deep_sleep_start();
}

// =====================================================
// LIQUID: PARTICLE FLUID ENGINE (unchanged)
// =====================================================

void initLiquid() {
  int idx = 0;

  for (int row = 0; row < LIQ_ROWS; row++) {
    for (int col = 0; col < LIQ_COLS; col++) {
      int xPix = 4 + col * 3 + (row & 1);
      int yPix = 61 - row * 3;

      px_[idx] = xPix * 256;
      py_[idx] = yPix * 256;
      ox_[idx] = px_[idx];
      oy_[idx] = py_[idx];
      vx_[idx] = 0;
      vy_[idx] = 0;
      idx++;
    }
  }

  simAccumulator = 0;
}

static inline void clampToWalls(int i) {
  if (px_[i] < MIN_X) px_[i] = MIN_X;
  else if (px_[i] > MAX_X) px_[i] = MAX_X;

  if (py_[i] < MIN_Y) py_[i] = MIN_Y;
  else if (py_[i] > MAX_Y) py_[i] = MAX_Y;
}

static void buildGrid() {
  memset(gridHead, 0xFF, sizeof(gridHead));   // all -1

  for (int i = 0; i < N; i++) {
    int cx = px_[i] >> 10;                    // /1024 = 4 px
    int cy = py_[i] >> 10;

    int cell = cy * GRID_W + cx;

    nextInCell[i] = gridHead[cell];
    gridHead[cell] = i;
  }
}

static void solveCollisions() {
  buildGrid();

  for (int i = 0; i < N; i++) {
    int cx = px_[i] >> 10;
    int cy = py_[i] >> 10;

    for (int gy2 = cy - 1; gy2 <= cy + 1; gy2++) {
      if (gy2 < 0 || gy2 >= GRID_H)
        continue;

      for (int gx2 = cx - 1; gx2 <= cx + 1; gx2++) {
        if (gx2 < 0 || gx2 >= GRID_W)
          continue;

        for (int j = gridHead[gy2 * GRID_W + gx2];
             j >= 0;
             j = nextInCell[j]) {

          if (j <= i)          // each pair only once
            continue;

          int32_t dx = px_[i] - px_[j];
          int32_t dy = py_[i] - py_[j];

          if (dx >= REST_DIST || dx <= -REST_DIST ||
              dy >= REST_DIST || dy <= -REST_DIST)
            continue;

          int32_t d2 = dx * dx + dy * dy;

          if (d2 >= REST_DIST2)
            continue;

          int32_t d;

          if (d2 == 0) {
            dx = (i & 1) ? 1 : -1;
            dy = 0;
            d = 1;
          } else {
            int32_t adx = dx < 0 ? -dx : dx;
            int32_t ady = dy < 0 ? -dy : dy;

            int32_t mx = adx > ady ? adx : ady;
            int32_t mn = adx > ady ? ady : adx;

            int32_t d0 = mx + ((mn * 3) >> 3);
            if (d0 < 1) d0 = 1;

            d = (d0 + d2 / d0) >> 1;
            if (d < 1) d = 1;
          }

          int32_t push = REST_DIST - d;       // overlap

          int32_t sx = dx * push / (2 * d);   // half each
          int32_t sy = dy * push / (2 * d);

          px_[i] += sx;
          py_[i] += sy;
          px_[j] -= sx;
          py_[j] -= sy;
        }
      }
    }
  }

  for (int i = 0; i < N; i++)
    clampToWalls(i);
}

static void getScreenGravity(float &gX, float &gY) {
  gX = -liquidGravityX;
  gY = liquidGravityY;

  if (LIQ_SWAP_XY) {
    float t = gX;
    gX = gY;
    gY = t;
  }

  if (LIQ_INVERT_X) gX = -gX;
  if (LIQ_INVERT_Y) gY = -gY;
}

static void simStep() {
  float gX, gY;
  getScreenGravity(gX, gY);

  gX = constrain(gX, -GRAVITY_CLAMP, GRAVITY_CLAMP);
  gY = constrain(gY, -GRAVITY_CLAMP, GRAVITY_CLAMP);

  float gMag = sqrtf(gX * gX + gY * gY);

  if (gMag < DESK_BIAS)
    gY += (DESK_BIAS - gMag);

  int32_t GX = (int32_t)(gX + (gX >= 0 ? 0.5f : -0.5f));
  int32_t GY = (int32_t)(gY + (gY >= 0 ? 0.5f : -0.5f));

  // 1) Integrate.
  for (int i = 0; i < N; i++) {
    vx_[i] += GX;
    vy_[i] += GY;

    if (vx_[i] > MAX_SPEED) vx_[i] = MAX_SPEED;
    else if (vx_[i] < -MAX_SPEED) vx_[i] = -MAX_SPEED;

    if (vy_[i] > MAX_SPEED) vy_[i] = MAX_SPEED;
    else if (vy_[i] < -MAX_SPEED) vy_[i] = -MAX_SPEED;

    ox_[i] = px_[i];
    oy_[i] = py_[i];

    px_[i] += vx_[i];
    py_[i] += vy_[i];

    clampToWalls(i);
  }

  // 2) Resolve overlaps.
  for (int it = 0; it < SOLVER_ITERS; it++)
    solveCollisions();

  // 3) Velocity from actual movement + light damping.
  for (int i = 0; i < N; i++) {
    int32_t vx = px_[i] - ox_[i];
    int32_t vy = py_[i] - oy_[i];

    vx -= vx >> DAMPING_SHIFT;
    vy -= vy >> DAMPING_SHIFT;

    if (vx > MAX_SPEED) vx = MAX_SPEED;
    else if (vx < -MAX_SPEED) vx = -MAX_SPEED;

    if (vy > MAX_SPEED) vy = MAX_SPEED;
    else if (vy < -MAX_SPEED) vy = -MAX_SPEED;

    vx_[i] = vx;
    vy_[i] = vy;
  }
}

static void splashKick(float strength) {
  float gX, gY;
  getScreenGravity(gX, gY);

  float gMag = sqrtf(gX * gX + gY * gY);

  float ux = 0.0f, uy = 1.0f;     // unit "down"

  if (gMag > 4.0f) {
    ux = gX / gMag;
    uy = gY / gMag;
  }

  for (int i = 0; i < N; i++) {
    if (random(100) >= 55)
      continue;

    float up = random(20, 90) * strength;

    vx_[i] += (int32_t)(-ux * up)
            + (int32_t)(random(-50, 51) * strength);

    vy_[i] += (int32_t)(-uy * up)
            + (int32_t)(random(-50, 51) * strength);
  }
}

void updateLiquid(float step) {
  bool newShake =
    shakeEnergy > 0.50f &&
    previousShakeEnergy <= 0.50f;

  if (newShake && millis() - lastSplash > 220) {
    float strength = constrain(
      0.8f + shakeEnergy * 0.8f,
      0.8f, 3.5f
    );

    splashKick(strength);
    lastSplash = millis();
  }

  previousShakeEnergy = shakeEnergy;

  simAccumulator += step;

  int steps = 0;

  while (simAccumulator >= SIM_DT && steps < MAX_SUBSTEPS) {
    simStep();
    simAccumulator -= SIM_DT;
    steps++;
  }

  if (simAccumulator > SIM_DT)
    simAccumulator = 0;
}

static inline void plotPixel(uint8_t *buf, int x, int y) {
  if ((unsigned)x > 127u || (unsigned)y > 63u)
    return;

  buf[(y >> 3) * 128 + x] |= (uint8_t)(1 << (y & 7));
}

void drawLiquid() {
  oled.clearBuffer();

  uint8_t *buf = oled.getBufferPtr();

  for (int i = 0; i < N; i++) {
    int x = (px_[i] + 128) >> 8;
    int y = (py_[i] + 128) >> 8;

    for (int oy = -1; oy <= 1; oy++) {
      plotPixel(buf, x - 1, y + oy);
      plotPixel(buf, x,     y + oy);
      plotPixel(buf, x + 1, y + oy);
    }
  }

  oled.sendBuffer();
}

// =====================================================
// FACE: BEHAVIOR
// =====================================================

void chooseGaze() {
  idleGazeX = random(-2, 3);
  idleGazeY = random(-1, 2);

  if (random(0, 7) == 0)
    idleGazeY = -2;

  nextLookTime = millis() + random(1600, 3400);
}

// Eyes pop open from a flat line (also used when switching
// from liquid mode to face mode).
void startWakeAnimation(unsigned long now) {
  wakeStart = now;
  wakeBlinkDone = false;

  sW   = {40.0f, 0};
  sH   = {3.0f,  0};
  sLid = {0.0f,  0};
  sSlant = {0.0f, 0};
  sRad = {1.0f,  0};

  blinking = false;
  extraBlinks = 0;
  buddyReacting = false;
  dizzy = false;
  dizzyMeter = 0;

  winkUntil = 0;
  heartUntil = 0;
  glanceUntil = 0;

  nextIdleEvent = now + random(2200, 4200);
}

void updateBuddy(float step) {
  unsigned long now = millis();
  unsigned long idleMs = now - lastActivity;

  // ---------- sensing ----------

  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  sway += (gyroMag - sway) * min(1.0f, step * 5.0f);

  // Sustained hard shaking -> dizzy.
  if (shakeEnergy > DIZZY_ENERGY) {
    dizzyMeter += step;
  } else {
    dizzyMeter -= step * 0.7f;
    if (dizzyMeter < 0) dizzyMeter = 0;
  }

  if (!dizzy && dizzyMeter > DIZZY_TIME) {
    dizzy = true;
    dizzyUntil = now + DIZZY_DURATION;
    dizzyMeter = 0;
    buddyReacting = false;
  }

  if (dizzy) {
    buddyReacting = false;

    if (now >= dizzyUntil) {
      dizzy = false;
      blinking = false;
      nextBlink = now + 600;
      chooseGaze();
    }
  }

  // Upside-down detection with hysteresis.
  // Screen "down" reading = -fx (same axis as the liquid).
  float downReading = FACE_UPSIDE_INVERT ? fx : -fx;

  if (downReading < -0.65f) {
    udTimer += step;
  } else if (downReading > -0.40f) {
    udTimer = 0;
    upsideDown = false;
  }

  if (udTimer > 0.35f)
    upsideDown = true;

  // Sleepy progress (0..1 over the last part of the idle timer).
  float sl = 0;

  if (mpuOK && idleMs > SLEEPY_START_MS) {
    sl = (float)(idleMs - SLEEPY_START_MS) /
         (float)(SLEEP_TIMEOUT - SLEEPY_START_MS);
    if (sl > 1.0f) sl = 1.0f;
  }

  sleepProgress = sl;

  // Existing shake reaction expiry.
  if (buddyReacting && now >= reactionUntil) {
    buddyReacting = false;

    chooseGaze();

    blinking = false;
    nextBlink = now + random(900, 2200);
  }

  // Gentle sway -> happy squint.
  bool gentle =
    sway > SWAY_MIN && sway < SWAY_MAX &&
    !buddyReacting && !dizzy;

  if (gentle) {
    squintTimer += step;
    if (squintTimer > 1.0f) squintTimer = 1.0f;
  } else {
    squintTimer -= step;
    if (squintTimer < 0) squintTimer = 0;
  }

  if (squintTimer > 0.45f)
    squintHold = now + 1400;

  // ---------- choose state (priority order) ----------

  unsigned long wakeT = now - wakeStart;
  bool waking = wakeT < WAKE_DURATION;

  FaceState s;

  if (dizzy)               s = FACE_DIZZY;
  else if (buddyReacting)  s = FACE_SURPRISED;
  else if (upsideDown)     s = FACE_WORRIED;
  else if (waking)         s = FACE_WAKING;
  else if (sl > 0.0f)      s = FACE_SLEEPY;
  else if (now < squintHold) s = FACE_SQUINT;
  else                     s = FACE_IDLE;

  faceState = s;

  if (SWAY_KEEPS_AWAKE && s == FACE_SQUINT)
    lastActivity = now;

  // ---------- idle extras: wink / hearts / glance / double blink ----------

  if (s != FACE_IDLE) {
    winkUntil = 0;
    heartUntil = 0;
    glanceUntil = 0;

    if (now >= nextIdleEvent)
      nextIdleEvent = now + random(1500, 3200);
  } else if (now >= nextIdleEvent) {
    int r = random(100);

    if (r < 30) {
      winkUntil = now + 520;
    } else if (r < 50) {
      heartUntil = now + 1400;
    } else if (r < 75) {
      glanceUntil = now + 800;
      glanceX = random(2) ? 4 : -4;
      glanceY = random(-1, 2);
    } else {
      extraBlinks = 1;
      blinking = true;
      blinkStart = now;
    }

    nextIdleEvent = now + random(2500, 5500);
  }

  // ---------- blinking ----------

  bool canBlink =
    (s == FACE_IDLE || s == FACE_WORRIED || s == FACE_WAKING);

  if (s == FACE_WAKING && !wakeBlinkDone && wakeT >= 850) {
    blinking = true;
    blinkStart = now;
    wakeBlinkDone = true;
  }

  if (canBlink) {
    if (s != FACE_WAKING && !blinking && now >= nextBlink) {
      blinking = true;
      blinkStart = now;
    }

    if (blinking && now - blinkStart > 135) {
      blinking = false;

      if (extraBlinks > 0) {
        extraBlinks--;
        nextBlink = now + 90;
      } else {
        nextBlink = now + random(1800, 4400);
      }
    }
  } else {
    blinking = false;
  }

  // ---------- gaze ----------

  if (s == FACE_IDLE && now >= nextLookTime)
    chooseGaze();

  float tiltX = constrain(fy * 2.5f, -2.0f, 2.0f);
  float tiltY = constrain(-fx * 1.5f, -1.5f, 1.5f);

  if (s == FACE_SURPRISED) {
    targetEyeOffsetX = tiltX * 0.3f;
    targetEyeOffsetY = tiltY * 0.3f;
  } else if (s == FACE_IDLE && now < glanceUntil) {
    targetEyeOffsetX = glanceX;
    targetEyeOffsetY = glanceY;
  } else if (s == FACE_SLEEPY) {
    targetEyeOffsetX = tiltX * 0.5f;
    targetEyeOffsetY = tiltY * 0.5f + sl * 1.5f;   // head sinks
  } else if (s == FACE_DIZZY) {
    targetEyeOffsetX = sinf(now * 0.012f) * 3.0f;
    targetEyeOffsetY = 0;
  } else if (s == FACE_WORRIED) {
    targetEyeOffsetX = tiltX;
    targetEyeOffsetY = tiltY;
  } else {
    targetEyeOffsetX = idleGazeX + tiltX;
    targetEyeOffsetY = idleGazeY + tiltY;
  }

  targetEyeOffsetX = constrain(targetEyeOffsetX, -4.0f, 4.0f);
  targetEyeOffsetY = constrain(targetEyeOffsetY, -3.0f, 3.0f);

  eyeOffsetX +=
    (targetEyeOffsetX - eyeOffsetX) * min(1.0f, step * 7.0f);

  eyeOffsetY +=
    (targetEyeOffsetY - eyeOffsetY) * min(1.0f, step * 7.0f);

  // ---------- eye shape targets ----------

  float tW = 35, tH = 35, tLid = 0, tSlant = 0, tRad = 7;

  yawnOpen = 0;

  switch (s) {
    case FACE_SURPRISED:
      tW = 37; tH = 40; tRad = 8;
      break;

    case FACE_WORRIED:
      tH = 33; tSlant = 9;
      break;

    case FACE_WAKING:
      if (wakeT < 250) {
        tW = 40; tH = 3; tRad = 1;
      }
      break;

    case FACE_SLEEPY: {
      tH = 35 - 6 * sl;
      tW = 35 + 2 * sl;
      tLid = 0.82f * sl;

      // One big yawn between 3.2 s and 4.4 s of inactivity.
      if (idleMs >= 3200 && idleMs <= 4400) {
        yawnOpen = sinf(PI * (idleMs - 3200) / 1200.0f);
        tLid += 0.18f * yawnOpen;
      }
      break;
    }

    default:
      break;
  }

  // Squash & stretch from vertical acceleration.
  if (s == FACE_IDLE || s == FACE_SQUINT || s == FACE_WORRIED) {
    float st = constrain((ax - fx) * 7.0f, -4.0f, 4.0f);
    tH += st;
    tW -= st * 0.6f;
  }

  springStep(sW,     tW,     step);
  springStep(sH,     tH,     step);
  springStep(sLid,   tLid,   step);
  springStep(sSlant, tSlant, step);
  springStep(sRad,   tRad,   step);

  if (s == FACE_SURPRISED)
    buddyBob = sinf(now * 0.025f) * 1.8f;
  else if (s == FACE_DIZZY)
    buddyBob = sinf(now * 0.02f) * 1.5f;
  else if (s == FACE_SQUINT)
    buddyBob = sinf(now * 0.008f) * 1.2f;
  else if (s == FACE_SLEEPY)
    buddyBob = sinf(now * 0.0016f) * 0.8f;
  else
    buddyBob = sinf(now * 0.0025f) * 0.5f;
}

// =====================================================
// FACE: DRAWING PRIMITIVES
// =====================================================

// Rounded eye with optional droopy lid and sad/worried slant.
void drawEyeBody(
  int x, int y, int w, int h,
  int rad, int lidPx, int slant,
  bool isLeft, bool blink
) {
  if (blink) {
    oled.drawRBox(x, y + h / 2 - 2, w, 5, 2);
    return;
  }

  int maxR = (min(w, h) - 1) / 2;
  int r = rad;
  if (r > maxR) r = maxR;
  if (r < 0) r = 0;

  if (r < 1)
    oled.drawBox(x, y, w, h);
  else
    oled.drawRBox(x, y, w, h, r);

  oled.setDrawColor(0);

  // Flat droopy eyelid.
  if (lidPx > 0)
    oled.drawBox(x - 1, y - 1, w + 2, lidPx + 1);

  // Sad / worried corner cut (outer top corner droops).
  if (slant > 0) {
    if (isLeft) {
      oled.drawTriangle(
        x - 1, y - 1,
        x + w + 1, y - 1,
        x - 1, y + slant
      );
    } else {
      oled.drawTriangle(
        x + w + 1, y - 1,
        x - 1, y - 1,
        x + w + 1, y + slant
      );
    }
  }

  oled.setDrawColor(1);
}

// Happy closed eye: a thick ^ arch. Used for squint and wink.
void drawArcEye(int cx, int cy, int wArc) {
  int x0 = cx - wArc / 2;
  int span = wArc - 1;

  for (int dx = 0; dx < wArc; dx++) {
    int hgt = (9 * 4 * dx * (span - dx)) / (span * span);
    int y0 = cy + 5 - hgt;

    oled.drawVLine(x0 + dx, y0, 5);
  }
}

void drawBlush(int bx, int by) {
  for (int i = 0; i < 3; i++)
    oled.drawLine(bx + i * 4, by + 3, bx + i * 4 + 2, by);
}

void drawHeart(int cx, int cy, float sc) {
  int r   = (int)(8 * sc + 0.5f);
  int dxh = (int)(7 * sc + 0.5f);
  int yo  = (int)(5 * sc + 0.5f);
  int tip = (int)(16 * sc + 0.5f);

  oled.drawDisc(cx - dxh, cy - yo, r);
  oled.drawDisc(cx + dxh, cy - yo, r);

  oled.drawTriangle(
    cx - dxh - r + 1, cy - yo + 2,
    cx + dxh + r - 1, cy - yo + 2,
    cx, cy + tip
  );
}

// Rotating spiral on a white disc (dizzy eye).
void drawSpiral(int cx, int cy, float phaseAngle) {
  const float COS_DA = 0.980067f;   // cos(0.2)
  const float SIN_DA = 0.198669f;   // sin(0.2)

  oled.drawDisc(cx, cy, 17);
  oled.setDrawColor(0);

  float c = cosf(phaseAngle);
  float s = sinf(phaseAngle);

  int prevX = cx + (int)lroundf(c * 1.0f);
  int prevY = cy + (int)lroundf(s * 1.0f);

  for (int i = 1; i < 64; i++) {
    float nc = c * COS_DA - s * SIN_DA;
    s = s * COS_DA + c * SIN_DA;
    c = nc;

    float rr = 1.0f + i * 0.23f;

    int x = cx + (int)lroundf(c * rr);
    int y = cy + (int)lroundf(s * rr);

    oled.drawLine(prevX, prevY, x, y);
    oled.drawLine(prevX + 1, prevY, x + 1, y);

    prevX = x;
    prevY = y;
  }

  oled.setDrawColor(1);
}

void drawZ(int x, int y, int s) {
  oled.drawHLine(x, y, s);
  oled.drawLine(x + s - 1, y, x, y + s - 1);
  oled.drawHLine(x, y + s - 1, s);
}

// Filled oval built from vertical lines (used for the yawn).
void fillOval(int cx, int cy, int rx, int ry) {
  if (rx < 1) rx = 1;

  for (int dx = -rx; dx <= rx; dx++) {
    float t = 1.0f - (float)(dx * dx) / (float)(rx * rx);
    if (t < 0) t = 0;

    int half = (int)(ry * sqrtf(t));

    oled.drawVLine(cx + dx, cy - half, 2 * half + 1);
  }
}

// ---------------- mouths ----------------

void drawSmile(int cx, int y) {
  oled.drawPixel(cx - 5, y);
  oled.drawHLine(cx - 4, y + 1, 3);
  oled.drawHLine(cx - 1, y + 2, 3);
  oled.drawHLine(cx + 2, y + 1, 3);
  oled.drawPixel(cx + 5, y);
}

void drawSurprisedMouth(int cx, int y) {
  oled.drawDisc(cx, y, 3);

  oled.setDrawColor(0);
  oled.drawDisc(cx, y, 1);
  oled.setDrawColor(1);
}

// smile = true: U shape, false: frown (n shape).
void drawArcMouth(int cx, int y, int hw, int depth, bool smile) {
  for (int dx = -hw; dx <= hw; dx++) {
    int off = (depth * dx * dx) / (hw * hw);
    int yy = smile ? (y + depth - off) : (y + off);

    oled.drawPixel(cx + dx, yy);
    oled.drawPixel(cx + dx, yy + 1);
  }
}

// Open happy mouth ("D" lying down): flat top, curved bottom.
void drawBowlMouth(int cx, int y) {
  for (int dx = -9; dx <= 9; dx++) {
    int len = 1 + (6 * (81 - dx * dx)) / 81;
    oled.drawVLine(cx + dx, y, len);
  }
}

void drawWavyMouth(int cx, int y, unsigned long now) {
  for (int dx = -8; dx <= 8; dx++) {
    int yy = y + 2 + (int)lroundf(sinf(dx * 0.7f + now * 0.02f) * 2.0f);

    oled.drawPixel(cx + dx, yy);
    oled.drawPixel(cx + dx, yy + 1);
  }
}

// =====================================================
// FACE: DISPLAY
// =====================================================

void drawBuddy() {
  oled.clearBuffer();

  unsigned long now = millis();
  FaceState s = faceState;

  int bob = (int)buddyBob;
  int eyeTop = 9 + bob + (int)eyeOffsetY;
  int centerY = eyeTop + 17;

  int w = clampInt((int)(sW.v + 0.5f), 20, 44);
  int h = clampInt((int)(sH.v + 0.5f), 3, 46);
  int rad = clampInt((int)(sRad.v + 0.5f), 0, 10);

  float lidFrac = constrain(sLid.v, 0.0f, 0.85f);
  int lidPx = (int)(lidFrac * h);

  int slant = clampInt((int)(sSlant.v + 0.5f), 0, 14);

  int leftCx  = 35 + (int)eyeOffsetX;
  int rightCx = 92 + (int)eyeOffsetX;

  int y = centerY - h / 2;
  if (y < 1) y = 1;

  int mouthY = eyeTop + 40;

  bool heart = (s == FACE_IDLE && now < heartUntil);
  bool wink  = (s == FACE_IDLE && now < winkUntil);

  // ---------- eyes ----------

  if (s == FACE_DIZZY) {
    float spin = now * 0.012f;

    drawSpiral(leftCx, centerY, spin);
    drawSpiral(rightCx, centerY, spin);
  } else if (s == FACE_SQUINT) {
    drawArcEye(leftCx, centerY, 31);
    drawArcEye(rightCx, centerY, 31);

    drawBlush(leftCx - 17, centerY + 12);
    drawBlush(rightCx + 6, centerY + 12);
  } else if (heart) {
    float sc = 1.0f + 0.12f * sinf(now * 0.02f);

    drawHeart(leftCx, centerY, sc);
    drawHeart(rightCx, centerY, sc);
  } else {
    drawEyeBody(
      leftCx - w / 2, y, w, h,
      rad, lidPx, slant, true, blinking
    );

    if (wink) {
      drawArcEye(rightCx, centerY, 28);
    } else {
      drawEyeBody(
        rightCx - w / 2, y, w, h,
        rad, lidPx, slant, false, blinking
      );
    }
  }

  // ---------- mouth ----------

  switch (s) {
    case FACE_SURPRISED:
      drawSurprisedMouth(64, mouthY);
      break;

    case FACE_WORRIED:
      drawArcMouth(64, mouthY + 2, 6, 3, false);
      break;

    case FACE_DIZZY:
      drawWavyMouth(64, mouthY, now);
      break;

    case FACE_SQUINT:
      drawBowlMouth(64, mouthY);
      break;

    case FACE_WAKING:
      oled.drawHLine(60, mouthY + 1, 9);
      break;

    case FACE_SLEEPY:
      if (yawnOpen > 0.05f) {
        fillOval(
          64, mouthY + 3,
          3 + (int)(4 * yawnOpen),
          1 + (int)(7 * yawnOpen)
        );
      } else {
        oled.drawHLine(61, mouthY + 1, 7);
      }
      break;

    default:   // FACE_IDLE
      if (wink || heart)
        drawArcMouth(64, mouthY - 1, 8, 3, true);
      else
        drawSmile(64, mouthY);
      break;
  }

  // ---------- floating z's while falling asleep ----------

  if (s == FACE_SLEEPY && sleepProgress > 0.45f) {
    for (int k = 0; k < 3; k++) {
      float age = fmodf(now * 0.0007f + k / 3.0f, 1.0f);

      int zx = 112 + (int)(age * 9);
      int zy = 24 - (int)(age * 22);
      int zs = 3 + (int)(age * 3);

      drawZ(zx, zy, zs);
    }
  }

  oled.sendBuffer();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("MIMO v0.1.0 booting...");

  esp_sleep_wakeup_cause_t cause =
    esp_sleep_get_wakeup_cause();

  Serial.printf("Wake-up cause: %d\n", (int)cause);

  if (cause == ESP_SLEEP_WAKEUP_GPIO)
    Serial.println("Wake-up source: GPIO motion interrupt");
  else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
    Serial.println("Cold boot or reset");

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(MPU_INT_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  oled.begin();
  oled.setPowerSave(0);
  oled.setContrast(255);

  randomSeed(micros());

  initLiquid();

  mpuOK = initMPU();

  switchLast = digitalRead(SWITCH_PIN);
  liquidMode = (switchLast == LOW);

  nextLookTime = millis() + 500;
  nextBlink = millis() + random(1500, 3500);

  startWakeAnimation(millis());

  lastActivity = millis();
  lastSensor = millis();
  lastFrame = millis();

  Serial.printf("MPU: %s\n", mpuOK ? "OK" : "ERROR");
  Serial.printf("Mode: %s\n", liquidMode ? "LIQUID" : "CUTEPY");
  Serial.printf("Liquid particles: %d\n", N);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  // ---------------------------------------------------
  // 1. Physical mode switch
  // ---------------------------------------------------

  bool currentSwitch = digitalRead(SWITCH_PIN);

  if (currentSwitch != switchLast) {
    switchLast = currentSwitch;
    liquidMode = (currentSwitch == LOW);

    lastActivity = now;

    if (liquidMode) {
      buddyReacting = false;
      simAccumulator = 0;
    } else {
      startWakeAnimation(now);   // face "wakes up" when selected
    }

    Serial.printf(
      "Mode changed: %s\n",
      liquidMode ? "LIQUID" : "CUTEPY"
    );
  }

  // ---------------------------------------------------
  // 2. Read MPU and detect physical activity
  // ---------------------------------------------------

  if (now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;

    if (mpuOK && !readMPU()) {
      mpuOK = false;
      Serial.println("MPU read failed!");
    }

    if (mpuOK)
      processMotion(now);
  }

  // ---------------------------------------------------
  // 3. Render selected mode
  // ---------------------------------------------------

  if (now - lastFrame >= FRAME_INTERVAL) {
    float step = constrain(
      (now - lastFrame) / 1000.0f,
      0.001f, 0.05f
    );

    lastFrame = now;

    if (liquidMode && mpuOK) {
      updateLiquid(step);
      drawLiquid();
    } else {
      updateBuddy(step);
      drawBuddy();
    }
  }

  // ---------------------------------------------------
  // 4. Shared inactivity timer
  // ---------------------------------------------------

  if (mpuOK && now - lastActivity >= SLEEP_TIMEOUT) {
    Serial.printf(
      "Idle timeout reached in %s mode.\n",
      liquidMode ? "LIQUID" : "CUTEPY"
    );

    enterDeepSleep();
    return;
  }
}
