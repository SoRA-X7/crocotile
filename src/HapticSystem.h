#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

const int sen_i2c_addr = 0x36;
const int sen_res = 12;
const int sen_angle_register_msb = 0x0e;
const int sen_bits_used_msb = 4;

struct HapticSystemConfig {
  // sensor
  int sen_pca_id;
  // driver
  int drv_uh;
  int drv_ul;
  int drv_vh;
  int drv_vl;
  int drv_wh;
  int drv_wl;
};

class HapticSystem {
 private:
  int sen_pca_id;
  MagneticSensorI2C sensor = MagneticSensorI2C(0, 0, 0, 0);
  BLDCMotor motor = BLDCMotor(0);
  BLDCDriver6PWM driver = BLDCDriver6PWM(0, 0, 0, 0, 0, 0);

  void pcaselect();

 public:
  HapticSystem(const HapticSystemConfig& cfg);
  ~HapticSystem();

  bool init();
  void loop();
  int feedback_mouse_position();
  float angle();
};
