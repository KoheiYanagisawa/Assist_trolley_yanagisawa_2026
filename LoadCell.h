#ifndef LOADCELL_H
#define LOADCELL_H

#include <Arduino.h>
#include "HX711.h"

class LoadCell {
  private:
    HX711 _scale;
    int _dout, _sck;
    float _calFactor;

  public:
    // コンストラクタ
    LoadCell(int dout, int sck, float calFactor);
    
    // 初期化と風袋引き（起動時に0点合わせ）
    void begin();
    
    // 現在の力（重さ）を取得
    float getForce();
    
    // 再度0点合わせを行う
    void tare();
};

#endif