#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  private:
    int _dirPin;
    int _pwmPin;

  public:
    // コンストラクタ：ピンの設定
    Motor(int dirPin, int pwmPin) {
      _dirPin = dirPin;
      _pwmPin = pwmPin;
      pinMode(_dirPin, OUTPUT);
      pinMode(_pwmPin, OUTPUT);
    }

    /**
     * モーターを駆動する
     * speed: -255 ～ 255 (負の数は逆転、正の数は正転)
     */
    void drive(int speed) {
      if (speed > 0) {
        digitalWrite(_dirPin, LOW);  // 正転
        analogWrite(_pwmPin, speed);
      } else if (speed < 0) {
        digitalWrite(_dirPin, HIGH); // 逆転
        analogWrite(_pwmPin, abs(speed));
      } else {
        analogWrite(_pwmPin, 0);     // 停止
      }
    }

    void stop() {
      drive(0);
    }
};

#endif