#pragma once

class VelocityEncoder {
    static int count1;
    static int count2;

    float speed1;
    float speed2;

    static void encoder1() {
      count1++;
    };
    static void encoder2() {
      count2++;
    };
  public:
    VelocityEncoder(int pin) {
      pinMode(pin, INPUT_PULLUP);
      if (pin == 2){
        count1 = 0;
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoder1, CHANGE);
      }
      if (pin == 3) {
        count2 = 0;
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoder2, CHANGE);
      }
    };
    ~VelocityEncoder() {};

    int every_second (int pin) {
      if (pin == 2){
        speed1 = count1 * 3.14 / 12;
        count1 = 0;
      }
      if (pin == 3) {
        speed2 = count2 * 3.14 / 12;
        count2 = 0;
      }
    };
    
    float getSpeed(int pin) {
      if (pin == 2){
        return speed1;
      }
      if (pin == 3) {
        return speed2;
      }
    };
};

int VelocityEncoder::count1 = 0;
int VelocityEncoder::count2 = 0;
