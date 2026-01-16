#include "LoadCell.h"

LoadCell::LoadCell(int dout, int sck, float calFactor) {
  _dout = dout;
  _sck = sck;
  _calFactor = calFactor;
}

void LoadCell::begin() {
  _scale.begin(_dout, _sck);
  _scale.set_scale(_calFactor);
  _scale.tare(); 
}

float LoadCell::getForce() {
  // サンプルの通り get_units(1) で1回のみ読み取り
  if (_scale.is_ready()) {
    return _scale.get_units(1);
  } else {
    return 0.0f;
  }
}

void LoadCell::tare() {
  _scale.tare();
}