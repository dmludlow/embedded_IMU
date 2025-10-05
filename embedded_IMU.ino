// Important conventions
// Scalar first quaternion
// All rotations are nav -> body
// SI units (m/s^2, rads, T)

#include <Wire.h>

const uint8_t MPU = 0x68;   // AD0=GND
const uint8_t MAG = 0x0C;   // AK8963

// Raw data (converted SI units)
float   accel[3], gyro[3], mag[3];
// Current attitude quaternion (scalar first convention)
float attitude_cur[4];
// Global bias variables (calulated during setup)
float gyro_bias[3];

// Conversion constants (for ±2g accel, ±250 dps gyro, 16-bit mag)
const float g_per_lsb    = 1.0f / 16384.0f;          // g per LSB
const float ms2_per_lsb  = 9.80665f * g_per_lsb;     // m/s^2 per LSB
const float dps_per_lsb  = 1.0f / 131.0f;            // deg/s per LSB
const float rads_per_lsb = (PI / 180.0f) * dps_per_lsb; // rad/s per LSB
const float ut_per_lsb   = 0.15f;                    // µT per LSB (AK8963, 16-bit mode)

// Coordinate frame definitions (normalized)
// NED convention to start, can change later if desired
float accel_world[3] = {0.0f, 0.0f, 1.0f};
// NED mag vector for Los Angeles
float mag_world[3] = {0.500f, 0.0f, 0.866f};

// Write helper function
void wr(uint8_t dev, uint8_t reg, uint8_t val){
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

// Read helper function
void rdb(uint8_t dev, uint8_t reg, uint8_t n, uint8_t* buf){
  Wire.beginTransmission(dev); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(dev, n, (uint8_t)true);
  for(uint8_t i=0; i<n; ++i) buf[i] = Wire.read();
}

// Helper function returns accel in SI units
void get_accel(float out[3]){
  uint8_t buf[6];
  rdb(MPU, 0x3B, 6, buf); // accel block starts at 0x3B

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

  out[0] = (float)ax * ms2_per_lsb;
  out[1] = (float)ay * ms2_per_lsb;
  out[2] = (float)az * ms2_per_lsb;
}

// Helper function returns gyro data in SI units
void get_gyro(float out[3]){
  uint8_t buf[6];
  rdb(MPU, 0x43, 6, buf); // gyro block starts at 0x43

  int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

  out[0] = (float)gx * rads_per_lsb;
  out[1] = (float)gy * rads_per_lsb;
  out[2] = (float)gz * rads_per_lsb;
}

// Helper function returns mag data in SI units
void get_mag(float out[3]){
  uint8_t buf[7];
  rdb(MAG, 0x03, 7, buf); // mag block: 6 data bytes + ST2

  int16_t mx = (int16_t)(buf[0] | (buf[1] << 8)); // little endian
  int16_t my = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t mz = (int16_t)(buf[4] | (buf[5] << 8));

  out[0] = (float)mx * ut_per_lsb * 1e-6f; // convert µT → Tesla
  out[1] = (float)my * ut_per_lsb * 1e-6f;
  out[2] = (float)mz * ut_per_lsb * 1e-6f;
}

// Normalize a 3D vector `in[3]` into `out[3]`
// Does not modify the original array
void v_norm(const float in[3], float out[3]) {
  float norm = sqrt(in[0]*in[0] + in[1]*in[1] + in[2]*in[2]);
  if (norm > 0.0f) {
    float inv = 1.0f / norm;
    out[0] = in[0] * inv;
    out[1] = in[1] * inv;
    out[2] = in[2] * inv;
  } else {
    // fallback: just copy input if norm=0
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
  }
}

// Normalize quaternion
void q_norm(float q[4]) {
  float norm = sqrt(q[0]*q[0] +
                    q[1]*q[1] +
                    q[2]*q[2] +
                    q[3]*q[3]);
  if (norm > 0.0f) {
    float inv = 1.0f / norm;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  // Small delay to allow sensor activation
  delay(200);

  // Wake MPU-9250
  wr(MPU, 0x6B, 0x00);      // PWR_MGMT_1 = 0

  // (optional) set full-scale ranges: ±2g, ±250 dps
  wr(MPU, 0x1C, 0x00);      // ACCEL_CONFIG
  wr(MPU, 0x1B, 0x00);      // GYRO_CONFIG

  // Enable I2C bypass to talk to AK8963 on 0x0C
  wr(MPU, 0x6A, 0x00);      // USER_CTRL: I2C_MST_EN=0
  wr(MPU, 0x37, 0x02);      // INT_PIN_CFG: BYPASS_EN=1

  // Init AK8963: power-down -> 16-bit continuous mode 2 (100 Hz)
  wr(MAG, 0x0A, 0x00); delay(10);   // CNTL1 = power-down
  wr(MAG, 0x0A, 0x16); delay(10);   // CNTL1 = 16-bit, continuous mode 2

  // Small delay to ensure chip is running
  delay(200);

  // Gryo bias calibration
  // Number of loops
  const int N = 1000;

  // Initialize running sums for average
  float gx = 0;
  float gy = 0;
  float gz = 0;

  for (int i = 0; i < N; i++){
    get_gyro(gyro);
    gx += gyro[0];
    gy += gyro[1];
    gz += gyro[2];
    // Sampling frequency at 500 Hz, with N = 1000 creates 2 second calibration period
    delay(2);
  }

  // Compute average and save bias
  gyro_bias[0] = gx / (float)N;
  gyro_bias[1] = gy / (float)N;
  gyro_bias[2] = gz / (float)N;

  // Determine initial attitude...
  float accel_body[3] = {0, 0, 0};
  float mag_body[3]   = {0, 0, 0};

  // Grab raw readings
  get_accel(accel);
  get_mag(mag);

  // Normalize raw readings and store in body frames
  v_norm(accel, accel_body);
  v_norm(mag, mag_body);

void loop() {
  // ---- ACCEL + GYRO (14 bytes starting at 0x3B) ----
  uint8_t buf14[14];
  rdb(MPU, 0x3B, 14, buf14);

}