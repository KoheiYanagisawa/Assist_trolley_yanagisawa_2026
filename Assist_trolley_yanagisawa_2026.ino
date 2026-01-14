#include <Wire.h>
#include "ACS712.h"
#include "Motor.h"
#include "Encoder.h"
#include "IMUManager.h"
#include "LoadCell.h"


const double KT_VALUE = 0.54;
// --- インスタンス作成 ---
// 電流センサ (ピン, 感度, オフセット, 平均回数)
ACS712 sensorL(A3, 13.0, 512.0, 9,KT_VALUE);
ACS712 sensorR(A2, 13.0, 512.0, 9,KT_VALUE);

// モーター (DIRピン, PWMピン)
Motor motorL(12, 11);
Motor motorR(10, 9);

//エンコーダー
Encoder encL(200.0, 0.065); // PPR, 直径
Encoder encR(200.0, 0.065, true); // 右は逆転

//IMU
IMUManager imu;

// ロードセル (DOUT, SCK, キャリブレーション係数)
// ピンは空いている22-25番などを使用
LoadCell handleL(22, 23, 2280.f);
LoadCell handleR(24, 25, 2280.f);

// --- 割り込み用ブリッジ ---
void isrL() { encL.tick(digitalRead(4) == LOW); }
void isrR() { encR.tick(digitalRead(6) == LOW); }

bool isRunning = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  handleL.begin();
  handleR.begin();

  attachInterrupt(digitalPinToInterrupt(2), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(3), isrR, RISING);
}

void loop() {
  // データの更新
  imu.update();
  double tL = sensorL.getTorque();
  double tR = sensorR.getTorque();
  double dL = encL.getDistance();
  double dR = encR.getDistance();

  // ハンドルを押す力を取得
  float forceL = handleL.getForce(); 
  float forceR = handleR.getForce();
  
  // IMUから3軸の角度を取得
  float roll  = imu.getRoll();
  float pitch = imu.getPitch();
  float yaw   = imu.getYaw();

  // シリアル出力 (タブ区切り)
  Serial.print(millis() / 1000.0, 2); Serial.print("\t");
  Serial.print(forceL, 2);            Serial.print("\t");
  Serial.print(forceR, 2);            Serial.print("\t");
  Serial.print(tL, 2);                Serial.print("\t");
  Serial.print(tR, 2);                Serial.print("\t");
  Serial.print(dL, 2);                Serial.print("\t");
  Serial.print(dR, 2);                Serial.print("\t");
  Serial.print(roll, 2);              Serial.print("\t");
  Serial.print(pitch, 2);             Serial.print("\t");
  Serial.println(yaw, 2);

  delay(20); 
}