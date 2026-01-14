#ifndef ACS712_H
#define ACS712_H
#include <Arduino.h>

class ACS712 {
  private:
    int _pin;
    double _sensitivity;
    double _offset;
    int _avgCount;
    double* _hist;
    int _index;
    double _kt; // トルク定数

  public:
    ACS712(int pin, double sensitivity, double offset, int avgCount, double kt);
    ~ACS712();
    double readCurrent();
    double getTorque(); // トルクを取得するメソッド
};
#endif