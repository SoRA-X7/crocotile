#pragma once
#include <BleMouse.h>
#include <SimpleFOC.h>

#include "HapticSystem.h"

#define MOUSE_LPF_TF 0.05
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
  LowPassFilter lpf_x = LowPassFilter(MOUSE_LPF_TF);
  LowPassFilter lpf_y = LowPassFilter(MOUSE_LPF_TF);
  BleMouse mouse;

 public:
  MouseDriver(float _sens, HapticSystem* _sysX, HapticSystem* _sysY,
              int pin_left, int pin_right);
  void init();
  void loop();
  void left_press();
  void left_release();
  void right_press();
  void right_release();
};
