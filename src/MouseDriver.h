#pragma once
#include <BleMouse.h>

#include "HapticSystem.h"

class MouseDriver {
 private:
  int prev_pos_x;
  int prev_pos_y;
  int pin_left;
  int pin_right;
  int left_pressed;
  int right_pressed;
  int sens;
  HapticSystem* sys_x;
  HapticSystem* sys_y;
  BleMouse mouse;

 public:
  MouseDriver(float _sens, HapticSystem* _sysX, HapticSystem* _sysY,
              int pin_left, int pin_right);
  void init();
  void loop();
  void left_click();
  void left_release();
  void right_click();
  void right_release();
};
