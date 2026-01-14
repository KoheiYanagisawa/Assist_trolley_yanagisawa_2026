#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
  private:
    volatile long _count;
    double _distPerPulse;
    bool _reverse;

  public:
    // コンストラクタ
    Encoder(double ppr, double diameter, bool reverse = false);
    
    // 割り込み用：カウントを増減させる
    void tick(bool direction);
    
    // カウント取得
    long getCount();
    // 距離[m]取得
    double getDistance();
    // リセット
    void reset();
};

#endif