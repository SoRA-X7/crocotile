#include "HapticSystem.h"

#include "Arduino.h"
#include "HardwareSerial.h"
#include "common/base_classes/FOCMotor.h"

#define PCAADDR 0x70

#define FOC_PID_P 4
#define FOC_PID_I 0
#define FOC_PID_D 0.04
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10

#define FOC_VOLTAGE_LIMIT 5

HapticSystem::HapticSystem(const HapticSystemConfig& cfg) {
  sen_pca_id = cfg.sen_pca_id;
  sensor = MagneticSensorI2C(sen_i2c_addr, sen_res, sen_angle_register_msb,
                             sen_bits_used_msb);
  motor = BLDCMotor(7);
  driver = BLDCDriver6PWM(cfg.drv_uh, cfg.drv_ul, cfg.drv_vh, cfg.drv_vl,
                          cfg.drv_wh, cfg.drv_wl);
}

HapticSystem::~HapticSystem() {}

bool HapticSystem::init() {
  pcaselect();
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 100;
  motor.useMonitoring(Serial);
  sensor.init();
  driver.voltage_power_supply = 5;
  driver.voltage_limit = 5;
  driver.pwm_frequency = 32000;

  if (!driver.init()) return false;

  motor.controller = MotionControlType::torque;
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // Configurable area
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.velocity_limit = 10000;
  motor.PID_velocity.P = FOC_PID_P;
  motor.PID_velocity.I = FOC_PID_I;
  motor.PID_velocity.D = FOC_PID_D;
  motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP;
  motor.PID_velocity.limit = FOC_PID_LIMIT;

  motor.init();
  motor.initFOC();

  delay(500);

  return true;
}

// void HapticSystem::loop() {
//   pcaselect();
//   motor.loopFOC();
//   const int split = 1;
//   float angle = motor.shaft_angle * 4;
//   long detent = round(angle);
//   float diff = angle - detent;
//   float torque = -diff * split;
//   torque = torque * torque * 5 * ((torque > 0) - (torque < 0));
//   motor.move(torque);
//   // motor.move(0);
// }

void HapticSystem::loop() {
  motor.loopFOC();
  if (bumps != NULL) {
    float dx = (angle() - mouseOffsetX) * 400;  // 400 = sens

    float strength = 0;
    if (dx < mouseMinX) {
      strength = 2 * (mouseMinX - dx);
    } else if (dx > mouseMaxX) {
      strength = -2 * (dx - mouseMaxX);
    } else {
      for (int i = 0; i < bump_count; i++) {
        int distance = dx - bumps[i].dx;
        if (distance == 0) continue;
        if (distance > 100) continue;
        strength += bumps[i].strength / (distance / 10.0);
      }
    }
    motor.move(strength * 0.5);
  } else {
    motor.move(0);
  }
  motor.monitor();
}

void HapticSystem::set_pattern(HapticPatternBump* bumps, int count, int min,
                               int max) {
  this->bumps = bumps;
  this->bump_count = count;
  this->mouseOffsetX = this->angle();
  this->mouseMinX = min;
  this->mouseMaxX = max;
}

void HapticSystem::revoke_pattern() {
  bumps = NULL;
  bump_count = 0;
}

float HapticSystem::angle() { return motor.shaft_angle; }

void HapticSystem::pcaselect() {
#if defined(__AVR_ATmega2560__) || defined(AVR_ATmega1280)
  uint8_t id = sen_pca_id;
  if (id > 7) return;

  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << id);
  Wire.endTransmission();
#endif
}
