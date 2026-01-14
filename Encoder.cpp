#include "Encoder.h"

Encoder::Encoder(double ppr, double diameter, bool reverse) {
  _count = 0;
  _reverse = reverse;
  // 1パルスあたりの距離 = (直径 * PI) / PPR
  _distPerPulse = (diameter * 3.14159265) / ppr;
}

void Encoder::tick(bool direction) {
  if (direction ^ _reverse) _count++;
  else _count--;
}

long Encoder::getCount() {
  long val;
  noInterrupts(); // 割り込み禁止で安全にコピー
  val = _count;
  interrupts();
  return val;
}

double Encoder::getDistance() {
  return getCount() * _distPerPulse;
}

void Encoder::reset() {
  noInterrupts();
  _count = 0;
  interrupts();
}