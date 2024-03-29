#include "MouseDriver.h"

MouseDriver* instance;

MouseDriver::MouseDriver(float _sens, HapticSystem* _sysX, HapticSystem* _sysY,
                         int _pin_left, int _pin_right) {
  sens = _sens;
  sys_x = _sysX;
  sys_y = _sysY;
  pin_left = _pin_left;
  pin_right = _pin_right;
}

void MouseDriver::init() {
  mouse.begin();
  prev_pos_x = (int)sys_x->angle() * sens;
  prev_pos_y = (int)sys_y->angle() * sens;

  pinMode(pin_left, INPUT_PULLUP);
  pinMode(pin_right, INPUT_PULLUP);
}

void MouseDriver::loop() {
  float diff_x = sys_x->angle() * sens - prev_pos_x;
  int div_x = (int)diff_x;
  prev_pos_x += div_x;
  float diff_y = sys_y->angle() * sens - prev_pos_y;
  int div_y = (int)diff_y;
  prev_pos_y += div_y;
  if (mouse.isConnected()) {
    // Serial.println("move");
    mouse.move(div_x, div_y, 0);

    if (!left_pressed && digitalRead(pin_left) == LOW) {
      left_press();
      left_pressed = true;
    } else if (left_pressed && digitalRead(pin_left) == HIGH) {
      left_release();
      left_pressed = false;
    }

    if (!right_pressed && digitalRead(pin_right) == LOW) {
      right_press();
      right_pressed = true;
    } else if (right_pressed && digitalRead(pin_right) == HIGH) {
      right_release();
      right_pressed = false;
    }
  }
}

void MouseDriver::left_press() {
  Serial.println("left_c");
  if (mouse.isConnected()) mouse.press(1);
}
void MouseDriver::left_release() {
  Serial.println("left_r");
  if (mouse.isConnected()) mouse.release(1);
}
void MouseDriver::right_press() {
  Serial.println("right_c");
  if (mouse.isConnected()) mouse.press(2);
}
void MouseDriver::right_release() {
  Serial.println("right_r");
  if (mouse.isConnected()) mouse.release(2);
}

void MouseDriver::set_sens(int _sens) { sens = _sens; }
