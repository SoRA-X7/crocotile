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
  direction = cfg.direction;
  sen_pca_id = cfg.sen_pca_id;
  sen_i2c_scl = cfg.sen_i2c_scl;
  sen_i2c_sda = cfg.sen_i2c_sda;
  sensor = MagneticSensorI2C(sen_i2c_addr, sen_res, sen_angle_register_msb,
                             sen_bits_used_msb);
  wire = TwoWire(cfg.sen_i2c_bus);
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
  Serial.print(sen_i2c_scl);
  Serial.println(sen_i2c_sda);
  wire.begin(sen_i2c_sda, sen_i2c_scl);
  sensor.init(&wire);
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

void HapticSystem::loop() {
  motor.loopFOC();
  if (haptic_pattern != NULL && haptic_pattern->bumps != NULL) {
    int neg = direction == 0 ? haptic_pattern->x_neg : haptic_pattern->x_pos;
    int pos = direction == 0 ? haptic_pattern->y_neg : haptic_pattern->y_pos;

    float dt = (angle() - mouseOffset) * 400;  // 400 = sens

    float strength = 0;
    int deadzone = 30;
    if (dt < neg - deadzone) {
      strength = 1 * (neg - dt);
    } else if (dt > pos + deadzone) {
      strength = -1 * (dt - pos);
    } else {
      for (int i = 0; i < haptic_pattern->bump_count; i++) {
        auto bump = haptic_pattern->bumps[i];
        int t = dt - (direction == 0 ? bump.dx : bump.dy);
        if (-bump.curve < t && t < 0) {
          strength += bump.strength * (-t - bump.curve) / bump.curve;
        } else if (0 < t && t < bump.curve) {
          strength += bump.strength * (-t + bump.curve) / bump.curve;
        }
        // strength += bumps[i].strength / (x / 10.0);
      }
    }
    motor.move(strength * 0.7);
  } else {
    motor.move(0);
  }
  motor.monitor();
}

void HapticSystem::set_pattern(HapticPattern* pat) {
  this->haptic_pattern = pat;
  this->mouseOffset = this->angle();
}

void HapticSystem::revoke_pattern() { this->haptic_pattern = NULL; }

void HapticSystem::revise_position(int x, int y) {
  int t = this->direction == 0 ? x : y;
  float dt = (angle() - mouseOffset) * 400;  // 400 = sens
  float diff = x - dt;
  mouseOffset -= diff / 400;
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
