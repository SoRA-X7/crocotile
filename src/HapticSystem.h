#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

const int sen_i2c_addr = 0x36;
const int sen_res = 12;
const int sen_angle_register_msb = 0x0e;
const int sen_bits_used_msb = 4;

struct HapticSystemConfig {
  int direction;  // 0 = x, 1 = y
  // sensor
  int sen_pca_id;
  int sen_i2c_bus;
  int sen_i2c_scl;
  int sen_i2c_sda;
  // driver
  int drv_uh;
  int drv_ul;
  int drv_vh;
  int drv_vl;
  int drv_wh;
  int drv_wl;
};

struct HapticPatternBump {
  int dx;
  int dy;
  float strength;
  float curve;
};

class HapticPattern {
 public:
  int x_neg;
  int x_pos;
  int y_neg;
  int y_pos;
  int intencity;
  int bump_count;
  HapticPatternBump* bumps;

  HapticPattern(int bump_count) {
    this->bump_count = bump_count;
    this->bumps =
        (HapticPatternBump*)malloc(bump_count * sizeof(HapticPatternBump));
  }
  ~HapticPattern() { free(this->bumps); }
};

class HapticSystem {
 private:
  int direction;
  int sen_pca_id;
  TwoWire wire = TwoWire(0);
  int sen_i2c_scl;
  int sen_i2c_sda;
  MagneticSensorI2C sensor = MagneticSensorI2C(0, 0, 0, 0);
  BLDCMotor motor = BLDCMotor(0);
  BLDCDriver6PWM driver = BLDCDriver6PWM(0, 0, 0, 0, 0, 0);
  HapticPattern* haptic_pattern;
  float mouseOffset;

  void pcaselect();

 public:
  HapticSystem(const HapticSystemConfig& cfg);
  ~HapticSystem();

  bool init();
  void loop();
  void set_pattern(HapticPattern* pat);
  void revoke_pattern();
  void revise_position(int x, int y);
  float angle();
};
