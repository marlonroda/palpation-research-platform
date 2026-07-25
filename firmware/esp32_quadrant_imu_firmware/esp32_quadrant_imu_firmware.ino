// ====== Quadrant Pad (MCP3008) + IMU on ESP32  ======
// - MCP3008 over software SPI
// - Kalman per quadrant
// - Force (quadrant) data at ~500 Hz
// - IMU data updated at ~200 Hz, but reused between force samples

#include <Adafruit_MCP3008.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>

// ---------- IMU CHOICE (pick ONE) ----------
#define USE_MPU6050   1   // set to 1 to use Adafruit_MPU6050 (default)
// #define USE_BNO08X    1   // uncomment to use Adafruit_BNO08x (and set USE_MPU6050 to 0)

// ---------- IMU LIBRARIES ----------
#if (defined(USE_MPU6050) && USE_MPU6050)
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  Adafruit_MPU6050 mpu;
#elif (defined(USE_BNO08X) && USE_BNO08X)
  #include <Adafruit_BNO08x.h>
  Adafruit_BNO08x bno08x;
  sh2_SensorValue_t sensorValue;
#else
  #warning "No IMU selected. Define USE_MPU6050 or USE_BNO08X."
#endif

// -------- Kalman filters: one per quadrant --------
SimpleKalmanFilter kTL(2, 2, 0.01f); // CH0
SimpleKalmanFilter kTR(2, 2, 0.01f); // CH1
SimpleKalmanFilter kBL(2, 2, 0.01f); // CH2
SimpleKalmanFilter kBR(2, 2, 0.01f); // CH3

// -------- Cadences (in microseconds) --------
// Target: IMU ~200 Hz, Force/print ~500 Hz
const uint32_t IMU_DT_US   = 5000;   // 1/200 s = 5 ms
const uint32_t FORCE_DT_US = 1429;   // 1/500 s = 2 ms

uint32_t nextImuUs   = 0;
uint32_t nextForceUs = 0;

// -------- ADC --------
Adafruit_MCP3008 adc;

// -------- Optional header toggle --------
const bool PRINT_HEADER = true;
bool headerPrinted = false;

// -------- IMU state buffers --------
#if (defined(USE_MPU6050) && USE_MPU6050)
// SI units from Adafruit_MPU6050: accel (m/s^2), gyro (rad/s)
volatile float ax = 0, ay = 0, az = 0;
volatile float gx = 0, gy = 0, gz = 0;
#elif (defined(USE_BNO08X) && USE_BNO08X)
// BNO08x rotation vector + accelerometer
volatile float qw = 0, qx = 0, qy = 0, qz = 0;
volatile float iax = 0, iay = 0, iaz = 0;
#endif

// --- Debug: measure print frequency (Lines Per Second) ---
//uint32_t dbgLastMs = 0;
//uint32_t dbgCount  = 0;

// ===================================================
// IMU helpers
// ===================================================
bool imu_begin() {
  // ESP32 default I2C pins: SDA=21, SCL=22
  Wire.begin(21, 22, 400000);
  delay(50);

#if (defined(USE_MPU6050) && USE_MPU6050)
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // ~20 Hz BW, 1 kHz ODR
  delay(100);
  return true;

#elif (defined(USE_BNO08X) && USE_BNO08X)
  if (!bno08x.begin()) return false;
  bno08x.enableReport(SH2_ROTATION_VECTOR,  5000);
  bno08x.enableReport(SH2_ACCELEROMETER,    5000);
  delay(50);
  return true;

#else
  return true; // no IMU selected — still run pad
#endif
}

// Poll IMU at its own rate (~200 Hz)
void imu_poll_if_due(uint32_t nowUs) {
  if ((int32_t)(nowUs - nextImuUs) < 0) return;
  nextImuUs += IMU_DT_US;

#if (defined(USE_MPU6050) && USE_MPU6050)
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  ax = a.acceleration.x; ay = a.acceleration.y; az = a.acceleration.z;
  gx = g.gyro.x;         gy = g.gyro.y;         gz = g.gyro.z;

#elif (defined(USE_BNO08X) && USE_BNO08X)
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      qw = sensorValue.un.rotationVector.real;
      qx = sensorValue.un.rotationVector.i;
      qy = sensorValue.un.rotationVector.j;
      qz = sensorValue.un.rotationVector.k;
    } else if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      iax = sensorValue.un.accelerometer.x;
      iay = sensorValue.un.accelerometer.y;
      iaz = sensorValue.un.accelerometer.z;
    }
  }
#endif
}

// ===================================================
// Setup
// ===================================================
void setup() {
  Serial.begin(921600);           // high baud for 500 Hz stream

  // MCP3008: software SPI (sck=18, mosi=23, miso=19, cs=25)
  adc.begin(18, 23, 19, 25);

  // Prime filters to avoid first-line jump
  for (int i = 0; i < 8; i++) {
    (void)kTL.updateEstimate(adc.readADC(0));
    (void)kTR.updateEstimate(adc.readADC(1));
    (void)kBL.updateEstimate(adc.readADC(2));
    (void)kBR.updateEstimate(adc.readADC(3));
    delay(5);
  }

  // IMU init (optional)
  if (!imu_begin()) {
    Serial.println("# WARN: IMU not found — continuing with pad only.");
  }

  uint32_t nowUs = micros();
  nextImuUs   = nowUs;
  nextForceUs = nowUs;
}

// ===================================================
// Loop
// ===================================================
void loop() {
  uint32_t nowUs = micros();

  // ---- IMU (poll at ~200 Hz) ----
  imu_poll_if_due(nowUs);

  // ---- Force sensor + print at ~500 Hz ----
  if ((int32_t)(nowUs - nextForceUs) >= 0) {
    nextForceUs += FORCE_DT_US;

    // Read raw channels
    float real_TL = adc.readADC(0);
    float real_TR = adc.readADC(1);
    float real_BL = adc.readADC(2);
    float real_BR = adc.readADC(3);

    // Kalman estimates (force-related)
    float est_TL = kTL.updateEstimate(real_TL);
    float est_TR = kTR.updateEstimate(real_TR);
    float est_BL = kBL.updateEstimate(real_BL);
    float est_BR = kBR.updateEstimate(real_BR);

    // --- Header (printed once) ---
    if (PRINT_HEADER && !headerPrinted) {
      // 10 columns: 4 force (est), 6 IMU
      Serial.print("# TL_est\tTR_est\tBL_est\tBR_est");
#if (defined(USE_MPU6050) && USE_MPU6050)
      Serial.print("\tax\tay\taz\tgx\tgy\tgz");
#elif (defined(USE_BNO08X) && USE_BNO08X)
      Serial.print("\tqw\tqx\tqy\tqz\tax\tay\taz");
#endif
      Serial.println();
      headerPrinted = true;
    }

    // --- Data line ---
    // Force (estimated quadrants)
    Serial.print(est_TL,2);  Serial.print('\t');
    Serial.print(est_TR,2);  Serial.print('\t');
    Serial.print(est_BL,2);  Serial.print('\t');
    Serial.print(est_BR,2);  Serial.print('\t');

    // IMU: use *latest* values (updated at 200 Hz)
#if (defined(USE_MPU6050) && USE_MPU6050)
    Serial.print(ax,2);  Serial.print('\t');
    Serial.print(ay,2);  Serial.print('\t');
    Serial.print(az,2);  Serial.print('\t');
    Serial.print(gx,2);  Serial.print('\t');
    Serial.print(gy,2);  Serial.print('\t');
    Serial.print(gz,2);
#elif (defined(USE_BNO08X) && USE_BNO08X)
    //Serial.print(qw);  Serial.print('\t');
    //Serial.print(qx);  Serial.print('\t');
    //Serial.print(qy);  Serial.print('\t');
    //Serial.print(qz);  Serial.print('\t');
    //Serial.print(iax); Serial.print('\t');
    //Serial.print(iay); Serial.print('\t');
    //Serial.print(iaz);
#endif

    Serial.println();

    // --- Debug frequency counter (lines per second) ---
    //dbgCount++;
    //uint32_t nowMs = millis();
    //if (nowMs - dbgLastMs >= 1000) {
    //  Serial.print("# LPS = ");  // Lines Per Second (≈ Hz)
    //  Serial.println(dbgCount);
    //  dbgCount  = 0;
    //  dbgLastMs = nowMs;
    //}
  }

  // (No delay() here: we rely on micros()-based scheduling)
}
