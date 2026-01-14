#include "ACS712.h"

ACS712::ACS712(int pin, double sensitivity, double offset, int avgCount, double kt) {
  _pin = pin;
  _sensitivity = sensitivity;
  _offset = offset;
  _avgCount = avgCount;
  _kt = kt; // トルク定数を設定
  _hist = new double[_avgCount];
  for (int i = 0; i < _avgCount; i++) _hist[i] = 0.0;
  _index = 0;
  pinMode(_pin, INPUT);
}

ACS712::~ACS712() { delete[] _hist; }

double ACS712::readCurrent() {
  int raw = analogRead(_pin);
  double current_now = (raw - _offset) / _sensitivity * (-1.0);
  _hist[_index] = current_now;
  _index = (_index + 1) % _avgCount;
  
  double sum = 0;
  for (int i = 0; i < _avgCount; i++) sum += _hist[i];
  return sum / (double)_avgCount;
}

double ACS712::getTorque() {
  // 現在の平均電流からトルクを計算
  return readCurrent() * _kt;
}