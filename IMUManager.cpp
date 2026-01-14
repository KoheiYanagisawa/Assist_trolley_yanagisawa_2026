#include "IMUManager.h"

IMUManager::IMUManager() : _roll(0), _pitch(0), _yaw(0), _gzOffset(0), _initialized(false) {}

void IMUManager::begin() {
  _mpu.initialize();
  
  // キャリブレーション：静止時の平均ノイズを計測
  long sumGz = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumGz += gz;
    delay(2);
  }
  _gzOffset = (float)sumGz / samples; // 平均値をオフセットとする
  
  _lastMicros = micros();
}

void IMUManager::update() {
  int16_t ax, ay, az, gx, gy, gz;
  _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1e-6;
  _lastMicros = now;
  if (dt <= 0 || dt > 0.1) return;

  // ヨー角の計算：オフセットを引いてから積分する
  float gz_d = (gz - _gzOffset) / 131.0;
  _yaw += gz_d * dt;

  // ロールとピッチの計算（前回のコードと同様）
  float denom = sqrt((float)ay * ay + (float)az * az);
  if (denom < 1e-6) return;
  float accRoll  = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, denom) * 180.0 / PI;
  float gx_d = gx / 131.0;
  float gy_d = gy / 131.0;

  if (!_initialized) {
    _roll = accRoll; _pitch = accPitch; _initialized = true;
  } else {
    const float alpha = 0.98;
    _roll  = alpha * (_roll  + gx_d * dt) + (1 - alpha) * accRoll;
    _pitch = alpha * (_pitch + gy_d * dt) + (1 - alpha) * accPitch;
  }
}