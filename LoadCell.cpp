#include "LoadCell.h"

LoadCell::LoadCell(int dout, int sck, float calFactor) {
  _dout = dout;
  _sck = sck;
  _calFactor = calFactor;
}

void LoadCell::begin() {
  _scale.begin(_dout, _sck);
  _scale.set_scale(_calFactor);
  _scale.tare(); // 起動時の重さを0にする
}

float LoadCell::getForce() {
  // 5回平均で取得（ループ速度を落としすぎないよう調整）
  if (_scale.is_ready()) {
    return _scale.get_units(5);
  } else {
    return 0.0f;
  }
}

void LoadCell::tare() {
  _scale.tare();
}