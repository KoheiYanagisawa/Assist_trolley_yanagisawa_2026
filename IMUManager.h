#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <MPU6050.h>

class IMUManager {
  private:
    MPU6050 _mpu;
    float _roll, _pitch, _yaw;
    float _gzOffset; // ヨー角のドリフト補正用
    unsigned long _lastMicros;
    bool _initialized;

  public:
    IMUManager();
    void begin();
    void update();
    float getRoll()  { return _roll; }
    float getPitch() { return _pitch; }
    float getYaw()   { return _yaw; }
};

#endif