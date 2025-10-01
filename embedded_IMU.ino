#include <Wire.h>

const uint8_t MPU = 0x68;   // AD0=GND
const uint8_t MAG = 0x0C;   // AK8963

int16_t accel[3], gyro[3], mag[3];  // <-- your 3 arrays

// minimal helpers
void wr(uint8_t dev, uint8_t reg, uint8_t val){
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void rdb(uint8_t dev, uint8_t reg, uint8_t n, uint8_t* buf){
  Wire.beginTransmission(dev); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(dev, n, (uint8_t)true);
  for(uint8_t i=0; i<n; ++i) buf[i] = Wire.read();
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(200);

  // Wake MPU-9250
  wr(MPU, 0x6B, 0x00);      // PWR_MGMT_1 = 0

  // Enable I2C bypass so we can talk directly to the AK8963 on 0x0C
  wr(MPU, 0x6A, 0x00);      // USER_CTRL: I2C_MST_EN=0
  wr(MPU, 0x37, 0x02);      // INT_PIN_CFG: BYPASS_EN=1

  // Init AK8963: power-down -> 16-bit continuous mode 2 (100 Hz)
  wr(MAG, 0x0A, 0x00); delay(10);   // CNTL1 = power-down
  wr(MAG, 0x0A, 0x16); delay(10);   // CNTL1 = 16-bit, continuous mode 2
}

void loop() {
  // ---- ACCEL + GYRO (14 bytes starting at 0x3B) ----
  uint8_t buf14[14];
  rdb(MPU, 0x3B, 14, buf14);

  // Accel (big-endian)
  accel[0] = (int16_t)((buf14[0] << 8) | buf14[1]);  // AX
  accel[1] = (int16_t)((buf14[2] << 8) | buf14[3]);  // AY
  accel[2] = (int16_t)((buf14[4] << 8) | buf14[5]);  // AZ

  // Convert to SI units
  //accel[0] = accel[0] / 16384 * 9.80665;
  //accel[1] = accel[1] / 16384 * 9.80665;
  //accel[2] = accel[2] / 16384 * 9.80665;

  // skip temp: buf14[6], buf14[7]

  // Gyro (big-endian)
  gyro[0] = (int16_t)((buf14[8]  << 8) | buf14[9]);   // GX
  gyro[1] = (int16_t)((buf14[10] << 8) | buf14[11]);  // GY
  gyro[2] = (int16_t)((buf14[12] << 8) | buf14[13]);  // GZ

  // Convert to SI untis
  gyro[0] = gyro[0] / 131.0 * (PI / 180.0);
  gyro[1] = gyro[1] / 131.0 * (PI / 180.0);
  gyro[2] = gyro[2] / 131.0 * (PI / 180.0);

  // ---- MAG (7 bytes starting at 0x03: L,H pairs + ST2) ----
  uint8_t buf7[7];
  rdb(MAG, 0x03, 7, buf7);

  // Magnetometer is LITTLE-endian (low byte first)
  mag[0] = (int16_t)(buf7[0] | (buf7[1] << 8));  // MX
  mag[1] = (int16_t)(buf7[2] | (buf7[3] << 8));  // MY
  mag[2] = (int16_t)(buf7[4] | (buf7[5] << 8));  // MZ
  // buf7[6] = ST2 (reading it clears data-ready latch)

  // ---- (Optional) print to verify ----
  Serial.print("A: ");
  Serial.print(accel[0]); Serial.print(' ');
  Serial.print(accel[1]); Serial.print(' ');
  Serial.print(accel[2]); Serial.print("   G: ");
  Serial.print(gyro[0]);  Serial.print(' ');
  Serial.print(gyro[1]);  Serial.print(' ');
  Serial.print(gyro[2]);  Serial.print("   M: ");
  Serial.print(mag[0]);   Serial.print(' ');
  Serial.print(mag[1]);   Serial.print(' ');
  Serial.println(mag[2]);

  delay(5); // ~200 Hz; remove or shrink if you want faster updates
}