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
    LoadCell(int dout, int sck, float calFactor);
    void begin();
    float getForce(); // get_units(1) を使用するように更新
    void tare();
};

#endif