#pragma once
#include <BleMouse.h>

#include "HapticSystem.h"

class MouseDriver {
 private:
  int prev_pos;
  int sens;
  HapticSystem* sysX;
  BleMouse mouse;

 public:
  MouseDriver(float _sens, HapticSystem* _sysX);
  void init();
  void loop();
};
