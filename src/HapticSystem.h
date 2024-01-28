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

struct HapticMotorCalibration {
  float zero_elec_offset;
  Direction sensor_direction;
  int motor_pole_pairs;
};

struct HapticPatternBump {
  int direction;
  int dt;
  float strength;
  float curve;
};

class HapticPattern {
 public:
  int x_min;
  int x_max;
  int y_min;
  int y_max;
  int intensity;
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
  int sens = 400;
  int sen_pca_id;
  TwoWire wire = TwoWire(0);
  int sen_i2c_scl;
  int sen_i2c_sda;
  MagneticSensorI2C sensor = MagneticSensorI2C(0, 0, 0, 0);
  LowPassFilter lpf = LowPassFilter(0.05);
  BLDCMotor motor = BLDCMotor(1);
  BLDCDriver6PWM driver = BLDCDriver6PWM(0, 0, 0, 0, 0, 0);
  HapticPattern* haptic_pattern;
  float mouseOffset;
  char buf_[72];

  void pcaselect();

 public:
  HapticSystem(const HapticSystemConfig& cfg,
               const HapticMotorCalibration& calib);
  ~HapticSystem();

  void calibrate();
  bool init();
  void loop();
  void set_sens(int _sens);
  void set_pattern(HapticPattern* pat);
  void revoke_pattern();
  void revise_position(int x, int y);
  float angle();
};
