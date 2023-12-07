#include "MouseDriver.h"

MouseDriver::MouseDriver(float _sens, HapticSystem* _sysX) {
  sens = _sens;
  sysX = _sysX;
  //   mouse = new BleMouse();
}

void MouseDriver::init() {
  mouse.begin();
  prev_pos = (int)sysX->angle() * sens;
}

void MouseDriver::loop() {
  float diff = sysX->angle() * sens - prev_pos;
  int div = (int)diff;
  // float rem = diff - div;
  prev_pos += div;
  if (mouse.isConnected() && div != 0) {
    // Serial.println("move");
    mouse.move(div, 0, 0);
  }
}
